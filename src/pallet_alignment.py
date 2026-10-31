"""ROS adapter for the experimental pallet alignment controller.

Observe mode never creates a velocity publisher. Execution requires measured TF,
body geometry, fresh images, active smoother/collision monitor, and --execute.
"""
import argparse
import math
from pathlib import Path
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import TwistStamped
from lifecycle_msgs.srv import GetState
from rcl_interfaces.srv import GetParameters
from sensor_msgs.msg import CameraInfo, CompressedImage
from std_srvs.srv import SetBool
from tf2_ros import Buffer, TransformException, TransformListener

from pallet_vision import (PoseGate, control_step, detect_openings, load_config,
                           quaternion_matrix, target_in_base, validate_motion_config)

DEFAULT_CONFIG = Path(__file__).resolve().parents[1] / 'config/pallet_alignment.yaml'


def wait_result(node, future, seconds=3.0):
    rclpy.spin_until_future_complete(node, future, timeout_sec=seconds)
    if not future.done() or future.exception() is not None:
        future.cancel()
        raise RuntimeError('ROS service timed out or failed')
    return future.result()


class PalletAligner(Node):
    def __init__(self, cfg, execute=False, show=False):
        super().__init__('pallet_alignment_test')
        self.cfg, self.execute, self.show = cfg, execute, show
        self.detector = detect_openings
        if cfg.get('detector', 'contour') == 'yolo_pose':
            from pallet_pose_detector import PalletPoseDetector
            self.detector = PalletPoseDetector(cfg)
        self.image = None
        self.info = None
        self.info_received = 0.0
        self.last_stamp = 0
        self.gate = PoseGate(cfg['stable_frames'])
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(CompressedImage, cfg['image_topic'], self.receive_image, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, cfg['camera_info_topic'], self.receive_info, qos_profile_sensor_data)
        self.publisher = None
        self.lifecycle_clients = {}
        self.lifecycle_pending = {}
        self.lifecycle_states = {}
        self.last_lifecycle_check = 0.0
        self.last_log = 0.0
        if execute:
            validate_motion_config(cfg)

    def receive_image(self, message):
        self.image = message, time.monotonic()

    def receive_info(self, message):
        self.info = message
        self.info_received = time.monotonic()

    def enable_camera(self):
        client = self.create_client(SetBool, self.cfg['camera_enable_service'])
        try:
            if not client.wait_for_service(timeout_sec=5.0):
                raise RuntimeError('Camera manager is unavailable; run it on Waffle first')
            request = SetBool.Request()
            request.data = True
            response = wait_result(self, client.call_async(request), 5.0)
            if not response.success:
                raise RuntimeError(response.message)
            self.get_logger().info(f'Camera: {response.message}')
        finally:
            self.destroy_client(client)

    def preflight_motion(self):
        """Read live configuration; never change Nav2 or bypass its collision checks."""
        checks = {
            '/velocity_smoother': {'enable_stamped_cmd_vel': True},
            '/collision_monitor': {'enable_stamped_cmd_vel': True,
                                   'cmd_vel_in_topic': 'cmd_vel_smoothed',
                                   'cmd_vel_out_topic': 'cmd_vel'},
        }
        for name, expected in checks.items():
            client = self.create_client(GetParameters, name+'/get_parameters')
            try:
                if not client.wait_for_service(timeout_sec=3.0):
                    raise RuntimeError(f'{name} parameter service unavailable')
                request = GetParameters.Request()
                request.names = list(expected)
                response = wait_result(self, client.call_async(request))
                if len(response.values) != len(expected):
                    raise RuntimeError(f'{name}: incomplete parameter response')
                for (key, value), actual in zip(expected.items(), response.values):
                    if isinstance(value, bool):
                        valid = actual.type == 1 and actual.bool_value == value
                    else:
                        valid = actual.type == 4 and actual.string_value.lstrip('/') == value
                    if not valid:
                        raise RuntimeError(f'{name}: unexpected {key}; verify velocity chain')
            finally:
                self.destroy_client(client)
            lifecycle = self.create_client(GetState, name+'/get_state')
            self.lifecycle_clients[name] = lifecycle
            if not lifecycle.wait_for_service(timeout_sec=3.0):
                raise RuntimeError(f'{name} lifecycle service unavailable')
            state = wait_result(self, lifecycle.call_async(GetState.Request()))
            if state.current_state.id != 3:
                raise RuntimeError(f'{name} must be active')
            self.lifecycle_states[name] = time.monotonic()
        for topic, node_name in ((self.cfg['cmd_vel_topic'], 'velocity_smoother'),
                                 ('/cmd_vel_smoothed', 'collision_monitor')):
            endpoints = self.get_subscriptions_info_by_topic(topic)
            if not any(e.node_name == node_name and e.topic_type == 'geometry_msgs/msg/TwistStamped'
                       for e in endpoints):
                raise RuntimeError(f'Missing TwistStamped subscriber {node_name} on {topic}')
        self.publisher = self.create_publisher(TwistStamped, self.cfg['cmd_vel_topic'], 1)

    def chain_active(self):
        now = time.monotonic()
        for name, future in list(self.lifecycle_pending.items()):
            if future.done():
                if future.exception() is None and future.result().current_state.id == 3:
                    self.lifecycle_states[name] = now
                else:
                    self.lifecycle_states[name] = 0.0
                del self.lifecycle_pending[name]
        if now-self.last_lifecycle_check > 0.5:
            for name, client in self.lifecycle_clients.items():
                if name not in self.lifecycle_pending:
                    self.lifecycle_pending[name] = client.call_async(GetState.Request())
            self.last_lifecycle_check = now
        return len(self.lifecycle_states) == 2 and all(now-t < 1.5 for t in self.lifecycle_states.values())

    def command(self, linear=0.0, angular=0.0):
        if self.publisher is not None and rclpy.ok():
            message = TwistStamped()
            message.header.stamp = self.get_clock().now().to_msg()
            message.header.frame_id = self.cfg['base_frame']
            message.twist.linear.x = float(linear)
            message.twist.angular.z = float(angular)
            self.publisher.publish(message)

    def image_fresh(self, message, received):
        age = (self.get_clock().now().nanoseconds-Time.from_msg(message.header.stamp).nanoseconds)*1e-9
        return (-0.05 <= age <= self.cfg['max_image_age'] and
                time.monotonic()-received <= self.cfg['max_image_age'])

    def process_image(self, message):
        info = self.info
        if info is None or time.monotonic()-self.info_received > self.cfg['max_info_age']:
            raise ValueError('Waiting for fresh CameraInfo')
        info_age = (self.get_clock().now().nanoseconds-Time.from_msg(info.header.stamp).nanoseconds)*1e-9
        if not -0.05 <= info_age <= self.cfg['max_info_age']:
            raise ValueError('CameraInfo timestamp is stale; check clock synchronization')
        if not message.header.frame_id or message.header.frame_id != info.header.frame_id:
            raise ValueError('Image/CameraInfo frame mismatch')
        if info.distortion_model not in ('plumb_bob', 'rational_polynomial'):
            raise ValueError('Unsupported camera distortion model')
        k, d = np.array(info.k).reshape(3, 3), np.array(info.d)
        if not np.isfinite(k).all() or not np.isfinite(d).all() or k[0, 0] <= 0 or k[1, 1] <= 0:
            raise ValueError('CameraInfo is not calibrated')
        if info.binning_x > 1 or info.binning_y > 1 or info.roi.x_offset or info.roi.y_offset:
            raise ValueError('Binning/ROI CameraInfo is not supported by this prototype')
        image = cv2.imdecode(np.frombuffer(message.data, np.uint8), cv2.IMREAD_COLOR)
        if image is None or image.shape[:2] != (info.height, info.width):
            raise ValueError('JPEG decode failed or calibration resolution mismatch')
        detection = self.detector(image, k, d, self.cfg)
        if self.show:
            h, w = image.shape[:2]
            x0, y0, x1, y1 = (int(v*s) for v, s in zip(self.cfg['roi'], (w, h, w, h)))
            cv2.rectangle(image, (x0, y0), (x1, y1), (0, 180, 255), 1)
            for points, confidence in getattr(self.detector, 'preview_points', []):
                for i, (point, score) in enumerate(zip(points, confidence)):
                    if score >= self.cfg.get('keypoint_confidence', 0.7):
                        pixel = tuple(np.rint(point).astype(int))
                        cv2.circle(image, pixel, 3, (0, 180, 255), 1)
                        cv2.putText(image, str(i), pixel, cv2.FONT_HERSHEY_SIMPLEX, .35, (0, 180, 255), 1)
            if detection is None and self.cfg.get('detector') == 'yolo_pose':
                cv2.putText(image, 'MODEL POINTS ONLY: pose rejected', (10, 25),
                            cv2.FONT_HERSHEY_SIMPLEX, .5, (0, 180, 255), 1)
            if detection is not None:
                for quad in detection.corners.reshape(2, 4, 2):
                    cv2.polylines(image, [quad.astype(np.int32)], True, (0, 255, 0), 2)
                cv2.putText(image, f'z={detection.translation[2]:.3f}m rms={detection.error:.2f}px',
                            (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            cv2.imshow('Pallet test: green must follow FRONT openings; q=stop', image)
            if cv2.waitKey(1) & 0xff == ord('q'):
                raise KeyboardInterrupt
        return detection

    def base_target(self, detection, message):
        transform = self.tf_buffer.lookup_transform(
            self.cfg['base_frame'], message.header.frame_id, Time.from_msg(message.header.stamp))
        q, p = transform.transform.rotation, transform.transform.translation
        rotation = quaternion_matrix([q.x, q.y, q.z, q.w])
        return target_in_base(detection, rotation, np.array([p.x, p.y, p.z]), self.cfg)

    def report(self, text):
        if time.monotonic()-self.last_log > 1.0:
            self.get_logger().info(text)
            self.last_log = time.monotonic()

    def run(self):
        start = time.monotonic()
        last_valid = None
        completed_frames = 0
        last_processed = 0.0
        self.get_logger().info('EXECUTE: alignment only, no insertion' if self.execute else
                               'OBSERVE: no navigation goal or velocity commands')
        try:
            if self.execute:
                self.preflight_motion()
            start = time.monotonic()
            while rclpy.ok() and time.monotonic()-start < self.cfg['alignment_timeout']:
                rclpy.spin_once(self, timeout_sec=0.02)
                now = time.monotonic()
                if self.execute and not self.chain_active():
                    raise RuntimeError('Velocity smoother/collision monitor state lost')
                if now-start > self.cfg['startup_timeout'] and last_valid is None:
                    raise RuntimeError('No stable pallet pose before startup timeout; tune detection in observe mode')
                if last_valid is not None and now-last_valid > self.cfg['lost_timeout']:
                    raise RuntimeError('Pallet pose lost; stopped')
                if self.image is None or not self.image_fresh(*self.image):
                    self.command()
                    self.gate.reset()
                    completed_frames = 0
                    self.report('Waiting for fresh image; verify ROS_DOMAIN_ID=11 and device clocks')
                    continue
                message, received = self.image
                stamp = Time.from_msg(message.header.stamp).nanoseconds
                if stamp == self.last_stamp:
                    # Never extend a velocity command on repeated image timestamps.
                    continue
                if stamp < self.last_stamp:
                    self.command()
                    self.gate.reset()
                    completed_frames = 0
                    self.report('Non-monotonic image timestamp')
                    continue
                if last_processed and now-last_processed > self.cfg['max_image_age']:
                    self.gate.reset()
                    completed_frames = 0
                self.last_stamp = stamp
                last_processed = now
                try:
                    detection = self.process_image(message)
                    if detection is None:
                        raise ValueError('No unique valid opening pose; check points, ROI, calibration and PnP fit')
                    stable = self.gate.accept(detection)
                    if not stable:
                        self.command()
                        completed_frames = 0
                        self.report(f'Collecting consistent poses: {self.gate.count}/{self.cfg["stable_frames"]}')
                        continue
                    if not self.image_fresh(message, received):
                        raise ValueError('Frame became stale during processing')
                    if self.cfg['body_front_x'] is None or not self.cfg['extrinsics_verified']:
                        if self.execute:
                            raise RuntimeError('Missing measured geometry')
                        last_valid = time.monotonic()
                        t = detection.translation
                        self.report(f'CAMERA estimate: right={t[0]:.3f}, down={t[1]:.3f}, forward={t[2]:.3f}m; '
                                    f'rms={detection.error:.2f}px. Body distance unavailable until TF is measured.')
                        continue
                    target = self.base_target(detection, message)
                    linear, angular, state = control_step(target, self.cfg)
                    last_valid = time.monotonic()
                    self.report(f'{state}: gap={target[3]:.3f}m, lateral={target[1]:+.3f}m, '
                                f'yaw={math.degrees(target[2]):+.1f}deg; proposed v={linear:.3f}, w={angular:.3f}')
                    if state in ('TOO_CLOSE', 'NEED_REPOSITION', 'INVALID'):
                        self.command()
                        completed_frames = 0
                        if self.execute:
                            raise RuntimeError(f'{state}: start again from a farther aligned staging position')
                        continue
                    completed_frames = completed_frames+1 if state == 'ALIGNED' else 0
                    self.command(linear, angular)
                    if completed_frames >= self.cfg['stable_frames']:
                        self.get_logger().info('Alignment target reached. Stopped; no insertion.')
                        return True
                except (ValueError, cv2.error, TransformException) as error:
                    self.command()
                    self.gate.reset()
                    completed_frames = 0
                    self.report(str(error))
            raise RuntimeError('Alignment/observation timeout')
        finally:
            # Repeated zeros improve delivery on exit. Crash/network-loss fallback
            # remains the existing robot-side smoother/driver watchdog.
            for _ in range(3):
                self.command()
                time.sleep(0.03)
            if self.show:
                cv2.destroyAllWindows()


def run_alignment(cfg, execute=False, show=False, enable_camera=False):
    node = PalletAligner(cfg, execute, show)
    try:
        if enable_camera:
            node.enable_camera()
        return node.run()
    finally:
        node.destroy_node()


def main():
    parser = argparse.ArgumentParser(description='Pallet vision preview; --execute enables alignment movement')
    parser.add_argument('--config', type=Path, default=DEFAULT_CONFIG)
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--show', action='store_true', help='OpenCV display (requires graphical desktop)')
    parser.add_argument('--enable-camera', action='store_true', help='Call the camera manager on Waffle')
    args = parser.parse_args()
    try:
        cfg = load_config(args.config)
        if args.execute:
            validate_motion_config(cfg)
    except (ValueError, OSError) as error:
        parser.error(str(error))
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    try:
        return 0 if run_alignment(cfg, args.execute, args.show, args.enable_camera) else 1
    except KeyboardInterrupt:
        return 130
    except RuntimeError as error:
        print(f'Pallet test stopped: {error}')
        return 1
    finally:
        rclpy.try_shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
