"""Offline regression tests for integrated bounded missions. Never runs ROS nodes."""
import math
from pathlib import Path
from types import SimpleNamespace
import sys
import unittest
from unittest.mock import MagicMock, patch

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
        node.cfg = self.cfg
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

    def test_default_configuration(self):
        trial.validate_trial_config(self.cfg)
        self.assertEqual(self.cfg['target_body_gap'], .310)
        self.assertEqual(self.cfg['fork_projection'], .152)
        self.assertTrue(self.cfg['pose_refine_lm'])
        for key, value in [('trial_max_motion_seconds', 4), ('trial_max_displacement', .1),
                           ('trial_max_yaw_change', 1), ('max_linear_speed', .025),
                           ('pose_refine_lm', False), ('trial_max_motion_seconds', float('nan'))]:
            with self.subTest(key=key), self.assertRaises(ValueError):
                trial.validate_trial_config(dict(self.cfg, **{key: value}))

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

    def test_time_and_displacement_caps(self):
        with patch.object(trial.time, 'monotonic', return_value=100):
            node = self.node();node.first = 97.
            with self.assertRaisesRegex(RuntimeError, 'time limit'):node.motion_guard()
            node = self.node();node.origin = (0,0,0)
            for pose in [(.036,0,0,100,0,0), (0,0,.17,100,0,0)]:
                node.odometry = pose
                with self.assertRaisesRegex(RuntimeError, 'travel cap'):node.motion_guard()

    def test_execute_command_bounds_zero_and_observe(self):
        with patch.object(trial.time, 'monotonic', return_value=100), \
             patch.object(trial.PalletAligner, 'command') as send:
            node = self.node();node.command(.01, .05)
            self.assertEqual(node.first,100)
            self.assertEqual(node.origin,(0,0,0))
            node.first = 97
            with self.assertRaises(RuntimeError):node.command(.01,0)
            send.assert_called_with()
            node.command()  # A zero remains allowed after timeout.
            node = self.node()
            with self.assertRaisesRegex(RuntimeError,'bounds'):node.command(.011,0)
            send.assert_called_with()
            node = self.node();node.execute = False;node.odometry = None
            node.command(.01,.05)
            self.assertIsNone(node.first)
            self.assertIsNone(node.origin)

    def test_guard_runs_even_without_a_new_detection(self):
        node = self.node();node.first = 97
        with patch.object(trial.time,'monotonic',return_value=100), self.assertRaisesRegex(RuntimeError,'time limit'):
            node.chain_active()

    def test_runner_cleanup_camera_and_stop_result(self):
        node = MagicMock();node.run.return_value = True;node.finish.return_value = True
        with patch.object(trial,'Trial',return_value=node):
            self.assertTrue(trial.run_trial(self.cfg,True,False,True))
        node.enable_camera.assert_called_once()
        node.finish.assert_called_once();node.destroy_node.assert_called_once()
        node.reset_mock();node.run.side_effect = RuntimeError('cap reached')
        with patch.object(trial,'Trial',return_value=node),self.assertRaisesRegex(RuntimeError,'cap reached'):
            trial.run_trial(self.cfg,True)
        node.finish.assert_called_once();node.destroy_node.assert_called_once()
        node.run.side_effect = None;node.finish.return_value = False
        with patch.object(trial,'Trial',return_value=node),self.assertRaisesRegex(RuntimeError,'confirm a stop'):
            trial.run_trial(self.cfg,True)


class MissionIntegrationTests(unittest.TestCase):
    def invoke(self, argv, nav_success=True, align_success=True):
        stack = __import__('contextlib').ExitStack()
        self.addCleanup(stack.close)
        stack.enter_context(patch.object(sys,'argv',['nav2_send_goal.py']+argv))
        stack.enter_context(patch.object(mission.rclpy,'init'))
        stack.enter_context(patch.object(mission.rclpy,'try_shutdown'))
        factory = stack.enter_context(patch.object(mission,'BasicNavigator'))
        nav = stack.enter_context(patch.object(mission,'navigate',return_value=nav_success))
        align = stack.enter_context(patch.object(trial,'run_trial',return_value=align_success))
        result = mission.main()
        return result,factory,nav,align

    def test_success_uses_default_trial_after_navigation(self):
        result,_,nav,align = self.invoke(['1','--align-pallet','--execute','--show','--enable-camera'])
        self.assertEqual(result,0);nav.assert_called_once();align.assert_called_once()
        cfg,execute,show,camera = align.call_args.args
        self.assertEqual(cfg['target_body_gap'],.310)
        self.assertEqual(cfg['trial_max_motion_seconds'],3)
        self.assertTrue(cfg['pose_refine_lm']);self.assertTrue(execute and show and camera)

    def test_failed_goal_never_enters_trial(self):
        result,_,_,align = self.invoke(['1','--align-pallet','--execute'],nav_success=False)
        self.assertEqual(result,1);align.assert_not_called()

    def test_observe_and_navigation_only_semantics_unchanged(self):
        result,factory,nav,align = self.invoke(['1','--align-pallet'])
        self.assertEqual(result,0);factory.assert_not_called();nav.assert_not_called()
        self.assertFalse(align.call_args.args[1])

    def test_navigation_only_does_not_start_trial(self):
        result,_,nav,align = self.invoke(['1'])
        self.assertEqual(result,0);nav.assert_called_once();align.assert_not_called()

    def test_trial_failure_does_not_report_mission_success(self):
        result,_,_,_ = self.invoke(['1','--align-pallet','--execute'],align_success=False)
        self.assertEqual(result,1)

    def test_invalid_trial_is_rejected_before_navigation(self):
        with patch.object(sys,'argv',['nav2_send_goal.py','1','--align-pallet','--execute']), \
             patch('pallet_vision.load_config',return_value={'detector':'contour'}), \
             patch.object(mission,'BasicNavigator') as factory, self.assertRaises(SystemExit):
            mission.main()
        factory.assert_not_called()


if __name__ == '__main__':unittest.main()
