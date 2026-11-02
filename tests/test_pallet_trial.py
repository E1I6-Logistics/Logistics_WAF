"""Offline regression tests for integrated bounded missions. Never runs ROS nodes."""
from pathlib import Path
from types import SimpleNamespace
import sys
import unittest
from unittest.mock import MagicMock, patch, call

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src'))
import pallet_alignment_trial as trial
import nav2_send_goal as mission
from pallet_vision import load_config


class TrialTests(unittest.TestCase):
    def setUp(self):
        self.cfg = load_config(ROOT/'config/trial.yaml')

    def node(self):
        node = object.__new__(trial.Trial)
        node.cfg = dict(self.cfg, alignment_mode='bounded')
        node.progress = None
        node.odometry = (0., 0., 0., 100., 0., 0.)
        node.sensor = (SimpleNamespace(torque=True), 100.)
        node.statuses = {}
        node.get_publishers_info_by_topic = lambda _: []
        node.first = None
        node.origin = None
        node.execute = True
        node.records = []
        node.start_time = 100.
        return node

    def test_sensor_action_and_teleop_guards(self):
        changes = [dict(odometry=None), dict(odometry=(0,0,0,99,0,0)),
                   dict(odometry=(float('nan'),0,0,100,0,0)),
                   dict(sensor=(SimpleNamespace(torque=False),100)),
                   dict(sensor=(SimpleNamespace(torque=True),99)),
                   dict(statuses={'nav': SimpleNamespace(status_list=[SimpleNamespace(status=2)])}),
                   dict(get_publishers_info_by_topic=lambda _: [SimpleNamespace(node_name='teleop_keyboard')])]
        with patch.object(trial.time, 'monotonic', return_value=100):
            self.node().motion_guard()
            for change in changes:
                node = self.node()
                for k,v in change.items():setattr(node,k,v)
                with self.subTest(change=change), self.assertRaises(RuntimeError):node.motion_guard()

    def test_runner_cleanup_camera_and_stop_result(self):
        node = MagicMock();node.run.return_value = True;node.finish.return_value = True
        with patch.object(trial,'Trial',return_value=node):
            self.assertTrue(trial.run_trial(self.cfg,True,False,True))
        node.enable_camera.assert_called_once()
        node.prepare_perception.assert_called_once()
        self.assertLess(node.mock_calls.index(call.prepare_perception()),
                        node.mock_calls.index(call.run()))
        node.finish.assert_called_once();node.destroy_node.assert_called_once()
        node.reset_mock();node.run.side_effect = RuntimeError('cap reached')
        with patch.object(trial,'Trial',return_value=node),self.assertRaisesRegex(RuntimeError,'cap reached'):
            trial.run_trial(self.cfg,True)
        node.finish.assert_called_once();node.destroy_node.assert_called_once()
        node.run.side_effect = None;node.finish.return_value = False
        with patch.object(trial,'Trial',return_value=node),self.assertRaisesRegex(RuntimeError,'confirm a stop'):
            trial.run_trial(self.cfg,True)

    def test_cold_preparation_discards_pose_without_starting_motion(self):
        node = self.node()
        node.publisher = None
        node.image = ('frame', 100.)
        node.image_fresh = lambda *_: True
        node.process_image = MagicMock(return_value=None)
        node.gate = MagicMock()
        node.get_logger = MagicMock()
        with patch.object(trial.rclpy, 'ok', return_value=True), \
             patch.object(trial.rclpy, 'spin_once'), \
             patch.object(trial.time, 'monotonic', side_effect=[100.,100.,100.,100.8,100.8]):
            node.prepare_perception()
        self.assertIsNone(node.image)
        self.assertIsNone(node.publisher)
        self.assertIsNone(node.first)
        node.gate.reset.assert_called_once()
        self.assertAlmostEqual(node.records[0]['perception_preparation_seconds'], .8)
        # Preparation never pretends to refresh odometry. The stale guard survives.
        with patch.object(trial.time, 'monotonic', return_value=100.8), \
             self.assertRaisesRegex(RuntimeError, 'Odometry'):
            node.motion_guard()

    def test_preparation_failure_prevents_run_and_still_cleans_up(self):
        node = MagicMock()
        node.prepare_perception.side_effect = RuntimeError('Perception preparation timed out')
        with patch.object(trial, 'Trial', return_value=node), \
             self.assertRaisesRegex(RuntimeError, 'preparation timed out'):
            trial.run_trial(self.cfg, True)
        node.run.assert_not_called()
        node.finish.assert_called_once()
        node.destroy_node.assert_called_once()


class MissionIntegrationTests(unittest.TestCase):
    def invoke(self, argv, nav_success=True, align_success=True):
        stack = __import__('contextlib').ExitStack()
        self.addCleanup(stack.close)
        stack.enter_context(patch.object(sys,'argv',['nav2_send_goal.py']+argv))
        stack.enter_context(patch.object(mission.rclpy,'init'))
        stack.enter_context(patch.object(mission.rclpy,'try_shutdown'))
        factory = stack.enter_context(patch.object(mission,'ExistingPoseNavigator'))
        nav = stack.enter_context(patch.object(mission,'navigate',return_value=nav_success))
        align = stack.enter_context(patch.object(trial,'run_trial',return_value=align_success))
        result = mission.main()
        return result,factory,nav,align

    def test_observe_does_not_create_navigator_or_execute_alignment(self):
        result,factory,nav,align = self.invoke(['1','--align-pallet'])
        self.assertEqual(result,0);factory.assert_not_called();nav.assert_not_called()
        self.assertFalse(align.call_args.args[1])

    def test_trial_failure_does_not_report_mission_success(self):
        result,_,_,_ = self.invoke(['1','--align-pallet','--execute'],align_success=False)
        self.assertEqual(result,1)

    def test_invalid_trial_is_rejected_before_navigation(self):
        with patch.object(sys,'argv',['nav2_send_goal.py','1','--align-pallet','--execute']), \
             patch('pallet_vision.load_config',return_value={'detector':'contour'}), \
             patch.object(mission,'ExistingPoseNavigator') as factory, self.assertRaises(SystemExit):
            mission.main()
        factory.assert_not_called()


class ExistingPoseTests(unittest.TestCase):
    def node(self, received=False):
        node = object.__new__(mission.ExistingPoseNavigator)
        node.initial_pose_received = received
        node.initial_pose_pub = MagicMock()
        node._waitForNodeToActivate = MagicMock()
        node.info = MagicMock()
        node.goToPose = MagicMock()
        return node

    def test_delayed_amcl_discovery_never_resets_pose(self):
        node = self.node()
        def receive(_node, timeout_sec):
            node.initial_pose_received = True
        with patch.object(mission.rclpy, 'ok', return_value=True), \
             patch.object(mission.rclpy, 'spin_once', side_effect=receive), \
             patch.object(mission.time, 'monotonic', return_value=100):
            node.waitUntilNav2Active()
        node.initial_pose_pub.publish.assert_not_called()
        self.assertEqual(node._waitForNodeToActivate.call_args_list,
                         [call('amcl'), call('bt_navigator')])

    def test_missing_amcl_aborts_before_goal_without_reset(self):
        node = self.node()
        with patch.object(mission.rclpy, 'ok', return_value=True), \
             patch.object(mission.time, 'monotonic', side_effect=[100,131]), \
             self.assertRaisesRegex(RuntimeError, 'AMCL'):
            mission.navigate(node, (1,2,90), 600)
        node.initial_pose_pub.publish.assert_not_called()
        node.goToPose.assert_not_called()

    def test_existing_pose_needs_no_reset_or_extra_wait(self):
        node = self.node(received=True)
        with patch.object(mission.rclpy, 'spin_once') as spin:
            node.waitUntilNav2Active()
        spin.assert_not_called()
        node.initial_pose_pub.publish.assert_not_called()


if __name__ == '__main__':unittest.main()
