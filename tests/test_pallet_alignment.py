"""Offline tests: synthetic optics, differential-drive control and mission gates.

Run with ROS sourced and the distro NumPy/OpenCV, see README_PALLET_ALIGNMENT.md.
No ROS node is constructed and no robot/network commands are sent.
"""
import copy
import math
from pathlib import Path
import sys
import unittest
from unittest.mock import MagicMock, patch

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src'))
from pallet_vision import (Detection, PoseGate, control_step, detect_openings, estimate_pose,
                           load_config, opening_model, target_in_base, validate_motion_config)


class GeometryTests(unittest.TestCase):
    def setUp(self):
        self.cfg = load_config(ROOT/'config/pallet_alignment.yaml')
        self.k = np.array([[500., 0, 320], [0, 500., 240], [0, 0, 1]])
        self.d = np.zeros(5)

    def projected(self, yaw=0.2, translation=(0.01, 0.04, 0.3)):
        rvec = np.array([0., yaw, 0.])
        points = cv2.projectPoints(opening_model(self.cfg), rvec,
                                  np.array(translation), self.k, self.d)[0].reshape(8, 2)
        return points

    def test_known_dimensions_and_pose(self):
        model = opening_model(self.cfg)
        self.assertAlmostEqual(model[4, 0]-model[0, 0], 0.056406)
        for yaw in (-0.3, 0.0, 0.3):
            det = estimate_pose(self.projected(yaw), self.k, self.d, self.cfg)
            self.assertIsNotNone(det)
            np.testing.assert_allclose(det.translation, [0.01, 0.04, 0.3], atol=1e-5)
            np.testing.assert_allclose(det.rotation, cv2.Rodrigues(np.array([0., yaw, 0.]))[0], atol=1e-4)

    def test_impossible_correspondence_rejected(self):
        points = self.projected()
        points[3] += [20, 30]
        self.assertIsNone(estimate_pose(points, self.k, self.d, self.cfg))

    def test_segmentation_of_synthetic_openings(self):
        image = np.full((480, 640, 3), 220, np.uint8)
        points = self.projected(0.0)
        for quad in points.reshape(2, 4, 2):
            cv2.fillConvexPoly(image, np.rint(quad).astype(np.int32), (20, 20, 20))
        det = detect_openings(image, self.k, self.d, self.cfg)
        self.assertIsNotNone(det)
        self.assertLess(abs(det.translation[2]-0.3), 0.015)
        self.assertIsNone(detect_openings(np.full_like(image, 220), self.k, self.d, self.cfg))

    def test_multiple_pairs_are_not_selected(self):
        image = np.full((480, 640, 3), 220, np.uint8)
        for shift in (-0.075, 0.075):
            points = self.projected(0.0, (shift, 0.04, 0.3))
            for quad in points.reshape(2, 4, 2):
                cv2.fillConvexPoly(image, np.rint(quad).astype(np.int32), (20, 20, 20))
        self.assertIsNone(detect_openings(image, self.k, self.d, self.cfg))

    def test_body_front_reference_includes_camera_offset(self):
        self.cfg['body_front_x'] = 0.14
        # Optical +z forward, +x right, +y down -> base +x,+(-y),+(-z).
        optical_to_base = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]])
        det = Detection(np.zeros((8, 2)), np.eye(3), np.array([0., 0.1, 0.26]), 0.)
        target = target_in_base(det, optical_to_base, np.array([0.05, 0, 0.15]), self.cfg)
        np.testing.assert_allclose(target, [0, 0, 0, 0.17], atol=1e-8)

    def test_motion_requires_measured_geometry(self):
        with self.assertRaises(ValueError):
            validate_motion_config(self.cfg)
        self.cfg['extrinsics_verified'] = True
        self.cfg['body_front_x'] = None
        with self.assertRaises(ValueError):
            validate_motion_config(self.cfg)
        self.cfg['body_front_x'] = 0.068
        validate_motion_config(self.cfg)

    def test_motion_rejects_fork_crossing_entrance(self):
        self.cfg.update(extrinsics_verified=True, body_front_x=.14,
                        target_body_gap=.17, min_body_gap=.16, near_guard_gap=.19)
        for updates in ({'target_body_gap': .14}, {'target_body_gap': .15},
                        {'min_body_gap': .148}, {'min_body_gap': .14}):
            cfg = dict(self.cfg, **updates)
            with self.subTest(updates=updates), self.assertRaisesRegex(ValueError, 'Pre-insertion'):
                validate_motion_config(cfg)

    def test_controller_stops_and_does_not_insert(self):
        for target, expected in [((0, 0, 0, .17), 'ALIGNED'),
                                 ((-.02, 0, 0, .15), 'TOO_CLOSE'),
                                 ((.01, .03, 0, .18), 'NEED_REPOSITION'),
                                 ((.01, 0, .15, .18), 'NEED_REPOSITION'),
                                 ((float('nan'), 0, 0, .3), 'INVALID')]:
            v, w, state = control_step(target, self.cfg)
            self.assertEqual((v, w, state), (0., 0., expected))

    def test_controller_simulated_approach(self):
        # Fixed pallet face x=0.7m. Start with lateral/heading offsets.
        for initial_y in (-0.015, 0, 0.015):
            x, y, yaw = 0., initial_y, 0.04
            state = None
            for _ in range(3000):
                c, s = math.cos(yaw), math.sin(yaw)
                rotation = np.array([[c, s], [-s, c]])
                center = rotation @ np.array([.7-x, -y])
                normal = rotation @ np.array([1., 0])
                goal = center-normal*(.14+self.cfg['target_body_gap'])
                gap = float((center-[.14, 0]) @ normal)
                v, w, state = control_step((*goal, -yaw, gap), self.cfg)
                self.assertGreaterEqual(v, 0)
                self.assertLessEqual(v, self.cfg['max_linear_speed'])
                self.assertLessEqual(abs(w), self.cfg['max_angular_speed'])
                if state != 'ALIGNING':
                    break
                x += v*c*.05
                y += v*s*.05
                yaw += w*.05
            self.assertEqual(state, 'ALIGNED')
            self.assertGreaterEqual(.7-x-.14, .165)

    def test_trial_forward_commands_survive_opencr_integer_conversion(self):
        cfg = load_config(ROOT/'config/trial.yaml')
        # Recorded final pose from the rotation-only 20261101_123003 trial,
        # plus a close approach where the proportional speed is below 1cm/s.
        for target in [(0.06375976211201928, 0.01109132047154,
                        -0.08233875910533345, 0.3728619137057924),
                       (.01, 0., 0., .32)]:
            with self.subTest(target=target):
                v, w, state = control_step(target, cfg)
                self.assertEqual(state, 'ALIGNING')
                self.assertEqual(v, .01)
                self.assertEqual(int(v*100), 1)
                self.assertLessEqual(v, cfg['max_linear_speed'])
                self.assertLessEqual(abs(w), cfg['max_angular_speed'])

    def test_forward_floor_preserves_rotation_only_and_stop_conditions(self):
        cfg = load_config(ROOT/'config/trial.yaml')
        v, w, state = control_step((.06, .05, -.09, .37), cfg)
        self.assertEqual((v, state), (0., 'ALIGNING'))
        self.assertNotEqual(w, 0.)
        for target, expected in [((0., 0., 0., .31), 'ALIGNED'),
                                 ((-.04, 0., 0., .27), 'TOO_CLOSE'),
                                 ((-.01, 0., 0., .33), 'NEED_REPOSITION')]:
            with self.subTest(target=target):
                self.assertEqual(control_step(target, cfg), (0., 0., expected))

    def test_forward_floor_cannot_exceed_configured_speed_limit(self):
        cfg = load_config(ROOT/'config/trial.yaml')
        cfg['max_linear_speed'] = .009
        self.assertEqual(control_step((.06, .01, 0., .37), cfg),
                         (0., 0., 'INVALID'))

    def test_pose_gate_requires_reacquisition_after_jump(self):
        gate = PoseGate(3)
        det = estimate_pose(self.projected(), self.k, self.d, self.cfg)
        self.assertFalse(gate.accept(det))
        self.assertFalse(gate.accept(det))
        self.assertTrue(gate.accept(det))
        other = copy.deepcopy(det)
        other.translation[0] += 0.1
        self.assertFalse(gate.accept(other))
        self.assertFalse(gate.accept(det))


class MissionTests(unittest.TestCase):
    def setUp(self):
        import nav2_send_goal
        self.module = nav2_send_goal
        self.nav = MagicMock()
        from builtin_interfaces.msg import Time
        self.nav.get_clock().now().to_msg.return_value = Time()
        lift = patch('pallet_alignment_trial.lift')  # No actuator in offline tests.
        lift.start()
        self.addCleanup(lift.stop)

    def test_rejected_and_failed_goals_do_not_report_success(self):
        self.nav.goToPose.return_value = False
        self.assertFalse(self.module.navigate(self.nav, (1, 2, 90), 1.))
        self.nav.goToPose.return_value = True
        self.nav.isTaskComplete.return_value = True
        for result in (self.module.TaskResult.FAILED, self.module.TaskResult.CANCELED):
            self.nav.getResult.return_value = result
            self.assertFalse(self.module.navigate(self.nav, (1, 2, 90), 1.))
        self.nav.getResult.return_value = self.module.TaskResult.SUCCEEDED
        self.assertTrue(self.module.navigate(self.nav, (1, 2, 90), 1.))
        goal = self.nav.goToPose.call_args.args[0]
        self.assertEqual(goal.pose.position.z, 0.0)

    def test_navigation_timeout_cancels(self):
        self.nav.isTaskComplete.return_value = False
        with patch.object(self.module.time, 'monotonic', side_effect=[0, 2]), \
                patch.object(self.module.rclpy, 'ok', return_value=True):
            with self.assertRaises(RuntimeError):
                self.module.navigate(self.nav, (1, 2, 90), 1.)
        self.nav.cancelTask.assert_called_once()

    def test_stale_frames_and_observe_mode_cannot_publish(self):
        from pallet_alignment import PalletAligner
        from sensor_msgs.msg import CompressedImage
        obj = MagicMock()
        obj.cfg = load_config(ROOT/'config/pallet_alignment.yaml')
        obj.get_clock().now().nanoseconds = 10_000_000_000
        msg = CompressedImage()
        msg.header.stamp.sec = 9
        self.assertFalse(PalletAligner.image_fresh(obj, msg, 100))
        msg.header.stamp.sec = 10
        with patch('pallet_alignment.time.monotonic', return_value=100):
            self.assertTrue(PalletAligner.image_fresh(obj, msg, 100))
            self.assertFalse(PalletAligner.image_fresh(obj, msg, 99))
        obj.publisher = None
        PalletAligner.command(obj, .025, .1)  # No publisher in observe mode.


if __name__ == '__main__':
    unittest.main()
