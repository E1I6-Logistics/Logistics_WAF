"""Fixed-target alignment: observe while stopped, move on odometry, re-observe.

Observe mode never publishes velocity. With --execute, a verified alignment is
followed by a straight fork insertion on odometry, a lift and a straight retreat.
The aligned map pose and insertion travel are kept so place_pallet can return there.
"""
import argparse
from datetime import datetime
import json
import math
from pathlib import Path
import time
from zoneinfo import ZoneInfo

import cv2
import numpy as np
import rclpy
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped
from action_msgs.msg import GoalStatusArray
from turtlebot3_msgs.msg import SensorState
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import TransformException

from pallet_alignment import PalletAligner, camera_off
from pallet_vision import load_config, validate_motion_config, quaternion_matrix, vertical_face_target
from pallet_alignment_control import (aligned, alignment_errors, fixed_pallet, pallet_in_odom,
                                      plan_moves, validate_fixed_target_config, wrap_angle)

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONFIG = ROOT / 'config/trial.yaml'
DEFAULT_OUTPUT = ROOT / 'output/pallet_alignment_trials'
# Longest Wi-Fi/DDS dropout ridden out at zero velocity before aborting (odom/sensor state, rear scan).
LINK_WAIT = 5.
INSERT_LIFT_DOWN = 8.  # s of lift DOWN while stopped right before the insertion drive


def validate_trial_config(cfg):
    validate_fixed_target_config(cfg)
    for key in ('lift_time', 'retreat_distance'):
        value = cfg.get(key)
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
            raise ValueError(f'{key} must be finite and positive')
    if (cfg.get('detector') != 'yolo_pose' or cfg.get('pose_refine_lm') is not True
            or cfg.get('vertical_face_fit') is not True):
        raise ValueError('Trial requires detector: yolo_pose, pose_refine_lm and vertical_face_fit: true')
    # OpenCR receives int(v*100): commands below 0.01 become zero.
    for key, ceiling in (('max_linear_speed', .010), ('insertion_speed', .040), ('max_angular_speed', .050)):
        value = cfg.get(key)
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not .01 <= value <= ceiling:
            raise ValueError(f'{key} must be within [0.01, {ceiling}]')


def lift(node, command, seconds, cancel=None):
    """Run the linear actuator UP/DOWN for a full stroke, then STOP (no position feedback)."""
    publisher = node.create_publisher(String, '/linear_actuator/cmd', 10)
    try:
        deadline = time.monotonic()+3
        while publisher.get_subscription_count() == 0:
            if time.monotonic() > deadline:
                raise RuntimeError('No /linear_actuator/cmd subscriber (linear_motor_node)')
            rclpy.spin_once(node, timeout_sec=.05)
        node.get_logger().info(f'Lift {command} for {seconds:.1f}s')
        publisher.publish(String(data=command))
        until = time.monotonic()+seconds
        while time.monotonic() < until:
            if cancel and cancel():
                raise RuntimeError('Canceled')
            rclpy.spin_once(node, timeout_sec=.05)
    finally:
        publisher.publish(String(data='STOP'))
        time.sleep(.2)


class Trial(PalletAligner):
    def __init__(self, cfg, execute=False, show=False):
        validate_trial_config(cfg)
        super().__init__(cfg, execute, show)
        self.odometry = None
        self.sensor = None
        self.statuses = {}
        self.records = []
        self.first = None
        self.origin = None
        self.start_time = time.monotonic()
        self.output_velocity = None
        self.scan = None
        self.min_stamp = 0
        self.cancel = None
        self.placement = None
        self.link_resumed = 0.
        self.create_subscription(LaserScan, '/scan', self.receive_scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self.receive_odom, qos_profile_sensor_data)
        self.create_subscription(SensorState, '/sensor_state', self.receive_sensor, qos_profile_sensor_data)
        self.create_subscription(TwistStamped, '/cmd_vel', self.receive_velocity, qos_profile_sensor_data)
        qos = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        for topic in ('/navigate_to_pose/_action/status', '/navigate_through_poses/_action/status',
                      '/dock_robot/_action/status', '/undock_robot/_action/status'):
            self.create_subscription(GoalStatusArray, topic,
                                     lambda m, t=topic: self.statuses.update({t: m}), qos)

    def receive_odom(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        self.odometry = (p.x, p.y, yaw, time.monotonic(),
                         msg.twist.twist.linear.x, msg.twist.twist.angular.z)

    def receive_sensor(self, msg):
        self.sensor = msg, time.monotonic()

    def receive_velocity(self, msg):
        self.output_velocity = msg, time.monotonic()

    def receive_scan(self, msg):
        self.scan = msg, time.monotonic()

    def scan_fresh(self):
        """Received within 0.3s and stamped within 0.3s of now."""
        if self.scan is None or time.monotonic()-self.scan[1] > .3:
            return False
        age = (self.get_clock().now().nanoseconds-Time.from_msg(self.scan[0].header.stamp).nanoseconds)*1e-9
        return -.05 <= age <= .3

    def check_reverse_clearance(self):
        """Additional rear check; commands still pass through collision_monitor."""
        if not self.scan_fresh():
            # Never reverse on an old scan: hold zero and wait out a dropout instead of aborting.
            self.get_logger().warning(f'Rear LaserScan stale: holding zero command up to {LINK_WAIT:.0f}s')
            started = time.monotonic()
            while not self.scan_fresh():
                if time.monotonic()-started > LINK_WAIT:
                    raise RuntimeError('Backoff requires fresh rear LaserScan')
                super().command()
                rclpy.spin_once(self, timeout_sec=.02)
            self.link_resumed = time.monotonic()
            self.records.append({'t': time.monotonic()-self.start_time, 'scan_pause': time.monotonic()-started})
        scan = self.scan[0]
        if (not all(math.isfinite(v) for v in (scan.angle_min, scan.angle_increment, scan.range_min, scan.range_max))
                or scan.angle_increment == 0 or not 0 <= scan.range_min < scan.range_max):
            raise RuntimeError('Backoff LaserScan metadata is invalid')
        try:
            tf = self.tf_buffer.lookup_transform(self.cfg['base_frame'], scan.header.frame_id,
                                                Time.from_msg(scan.header.stamp)).transform
        except TransformException as exc:
            raise RuntimeError('Backoff requires scan-to-base TF') from exc
        q, p = tf.rotation, tf.translation
        if not all(math.isfinite(v) for v in (p.x, p.y, p.z)):
            raise RuntimeError('Backoff scan-to-base translation is invalid')
        rotation = quaternion_matrix([q.x, q.y, q.z, q.w])
        ranges = np.asarray(scan.ranges, dtype=float)
        angles = scan.angle_min+np.arange(len(ranges))*scan.angle_increment
        rays = np.column_stack((np.cos(angles), np.sin(angles), np.zeros(len(angles))))@rotation.T
        bearings = np.arctan2(rays[:, 1], -rays[:, 0])
        rear = np.abs(bearings) <= math.radians(60)
        valid = (np.isfinite(ranges) & (ranges >= scan.range_min) & (ranges <= scan.range_max)) | np.isposinf(ranges)
        covered = np.sort(bearings[rear & valid])
        if (scan.range_max < .5 or len(covered) < 20
                or covered[0] > -math.radians(55) or covered[-1] < math.radians(55)
                or np.max(np.diff(covered)) > math.radians(5)):
            raise RuntimeError('Backoff rear scan coverage is incomplete')
        finite = np.isfinite(ranges) & (ranges >= scan.range_min) & (ranges <= scan.range_max)
        points = rays[finite]*ranges[finite, None]+[p.x, p.y, p.z]
        # URDF rear face is x=-0.197m. Check an extra ~0.20m behind it,
        # and a 0.40m-wide corridor including wheels and lateral clearance.
        blocked = (points[:, 0] < 0) & (points[:, 0] > -.40) & (np.abs(points[:, 1]) < .20)
        if np.any(blocked):
            raise RuntimeError('Backoff blocked by rear obstacle')

    def map_pose(self):
        """base_frame (x, y, yaw) in map from AMCL's TF."""
        try:
            transform = self.tf_buffer.lookup_transform('map', self.cfg['base_frame'], Time()).transform
        except TransformException as exc:
            raise RuntimeError('Map pose requires map-to-base TF (AMCL)') from exc
        p, q = transform.translation, transform.rotation
        return p.x, p.y, math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))

    def prepare_perception(self):
        """Pay cold inference/GUI costs before creating a velocity publisher.

        Use the real image size and calibration, but discard this detection.
        Motion preflight subsequently spins and checks fresh sensor data again.
        """
        if self.publisher is not None:
            raise RuntimeError('Perception preparation must precede motion preflight')
        self.get_logger().info('Preparing model/display before alignment; no velocity publisher')
        deadline = time.monotonic()+self.cfg['startup_timeout']
        last_error = 'Waiting for fresh image and CameraInfo'
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=.05)
            if self.image is None or not self.image_fresh(*self.image):
                continue
            message, _ = self.image
            started = time.monotonic()
            try:
                # A rejected pose is fine here: this only initializes inference/GUI.
                self.process_image(message)
            except (ValueError, cv2.error) as exc:
                last_error = str(exc)
                continue
            elapsed = time.monotonic()-started
            self.records.append({'t': time.monotonic()-self.start_time,
                                 'perception_preparation_seconds': elapsed})
            self.image = None
            self.gate.reset()
            self.get_logger().info(f'Model/display ready ({elapsed:.3f}s); fresh poses required for alignment')
            return
        raise RuntimeError(f'Perception preparation timed out: {last_error}')

    def link_fresh(self):
        now = time.monotonic()
        return bool(self.odometry and now-self.odometry[3] <= .5 and self.sensor and now-self.sensor[1] <= .5)

    def wait_for_link(self):
        """Wi-Fi/DDS dropouts: hold zero commands up to LINK_WAIT for fresh odom/sensor state, then resume."""
        if self.odometry is None or self.link_fresh():
            return
        self.get_logger().warning(f'Odometry/sensor state stale: holding zero command up to {LINK_WAIT:.0f}s')
        started = time.monotonic()
        while time.monotonic()-started < LINK_WAIT:
            super().command()
            rclpy.spin_once(self, timeout_sec=.02)
            if self.link_fresh():
                # The pause is not a drive stall; move() restarts its stall timer from here.
                self.link_resumed = time.monotonic()
                self.records.append({'t': time.monotonic()-self.start_time, 'link_pause': time.monotonic()-started})
                self.get_logger().info(f'Link recovered after {time.monotonic()-started:.2f}s; resuming')
                return

    def motion_guard(self):
        self.wait_for_link()
        now = time.monotonic()
        if not self.odometry or not all(math.isfinite(v) for v in self.odometry) or now-self.odometry[3] > .5:
            raise RuntimeError('Odometry unavailable, invalid or stale')
        if not self.sensor or now-self.sensor[1] > .5:
            raise RuntimeError('Motor sensor state unavailable or stale')
        if not self.sensor[0].torque:
            raise RuntimeError('Motor torque disabled')
        if any(s.status in (1, 2, 3) for m in self.statuses.values() for s in m.status_list):
            raise RuntimeError('Another navigation/docking action is active')
        if any(e.node_name == 'teleop_keyboard' for e in self.get_publishers_info_by_topic('/cmd_vel')):
            raise RuntimeError('Teleop publisher still connected')

    def stationary(self):
        return bool(self.odometry and all(math.isfinite(v) for v in self.odometry)
                    and time.monotonic()-self.odometry[3] < .5
                    and abs(self.odometry[4]) < .005 and abs(self.odometry[5]) < .02)

    def preflight_motion(self):
        until = time.monotonic()+3
        while time.monotonic() < until:
            rclpy.spin_once(self, timeout_sec=.05)
        self.motion_guard()
        if not self.stationary():
            raise RuntimeError('Robot is already moving')
        # A negative command must not be silently clamped by the smoother.
        from pallet_alignment import wait_result
        from rcl_interfaces.srv import GetParameters
        client = self.create_client(GetParameters, '/velocity_smoother/get_parameters')
        try:
            if not client.wait_for_service(timeout_sec=3.):
                raise RuntimeError('Velocity smoother parameter service unavailable')
            request = GetParameters.Request()
            request.names = ['min_velocity', 'deadband_velocity']
            values = wait_result(self, client.call_async(request)).values
            if (len(values) != 2 or any(v.type != 8 or len(v.double_array_value) != 3 for v in values)
                    or not math.isfinite(values[0].double_array_value[0])
                    or values[0].double_array_value[0] > -self.cfg['insertion_speed']
                    or not 0 <= values[1].double_array_value[0] < self.cfg['max_linear_speed']):
                raise RuntimeError('Velocity smoother settings do not permit reverse speed')
        finally:
            self.destroy_client(client)
        super().preflight_motion()

    def chain_active(self):
        if self.cancel and self.cancel():
            raise RuntimeError('Canceled')
        # Check sensors even without a new detection.
        if self.execute:
            self.motion_guard()
        return super().chain_active()

    def command(self, linear=0., angular=0.):
        if self.execute and (linear or angular):
            try:
                self.motion_guard()
                if (not math.isfinite(linear) or not math.isfinite(angular)
                        or abs(linear) > max(self.cfg['max_linear_speed'], self.cfg['insertion_speed'])+1e-9
                        or abs(angular) > self.cfg['max_angular_speed']+1e-9):
                    raise RuntimeError('Command exceeds trial bounds')
                if linear < 0:
                    if abs(angular) > 1e-9:
                        raise RuntimeError('Backoff must be straight')
                    self.check_reverse_clearance()
            except RuntimeError:
                super().command()
                raise
            if self.first is None:
                self.first, self.origin = time.monotonic(), self.odometry[:3]
            self.records.append({'t': time.monotonic()-self.start_time,
                                 'v': linear, 'w': angular, 'odom': self.odometry})
        super().command(linear, angular)

    def base_target(self, detection, message):
        transform = self.tf_buffer.lookup_transform(
            self.cfg['base_frame'], message.header.frame_id, Time.from_msg(message.header.stamp))
        q, p = transform.transform.rotation, transform.transform.translation
        target = vertical_face_target(detection, *self.camera_model, quaternion_matrix([q.x, q.y, q.z, q.w]),
                                      np.array([p.x, p.y, p.z]), self.cfg)
        self.records.append({'t': time.monotonic()-self.start_time, 'target': target, 'rms_px': detection.error})
        return target

    def next_target(self, deadline):
        """Base target from the next fresh frame captured after min_stamp; None if rejected."""
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=.02)
            if self.execute and not self.chain_active():
                raise RuntimeError('Velocity smoother/collision monitor state lost')
            if self.image is None or not self.image_fresh(*self.image):
                continue
            message, received = self.image
            self.image = None
            if Time.from_msg(message.header.stamp).nanoseconds <= self.min_stamp:
                continue
            try:
                detection = self.process_image(message)
                if detection is None:
                    raise ValueError('No unique valid opening pose; check points, ROI, calibration and PnP fit')
                if not self.image_fresh(message, received):
                    raise ValueError('Frame became stale during processing')
                return self.base_target(detection, message)
            except (ValueError, cv2.error, TransformException) as error:
                self.report(str(error))
                return None
        return None

    def observe(self):
        """Pallet (x, y, yaw) in odom from consistent frames taken while stopped."""
        deadline = time.monotonic()+self.cfg['startup_timeout']
        samples = []
        while time.monotonic() < deadline:
            target = self.next_target(deadline)
            if target is None or not self.stationary():
                continue
            samples = samples[1-self.cfg['stable_frames']:]+[pallet_in_odom(target, self.odometry, self.cfg)]
            if len(samples) == self.cfg['stable_frames']:
                pallet = fixed_pallet(samples, self.cfg)
                if pallet is not None:
                    return pallet
            self.report(f'Observing while stopped: {len(samples)}/{self.cfg["stable_frames"]} frames')
        raise RuntimeError('No consistent pallet observation while stopped (odometry required)')

    def move(self, kind, value, deadline, speed=None):
        """Turn to an odom heading, or drive a signed straight distance; then settle."""
        linear = speed or self.cfg['max_linear_speed']
        start = self.odometry[:2]
        best, best_time = 0., time.monotonic()
        while True:
            if time.monotonic() > deadline:
                raise RuntimeError('Alignment timeout')
            rclpy.spin_once(self, timeout_sec=.02)
            if not self.chain_active():
                raise RuntimeError('Velocity smoother/collision monitor state lost')
            if kind == 'turn':
                error = wrap_angle(value-self.odometry[2])
                # 0.3deg moves the fork line ~2mm at the pallet face.
                if abs(error) <= math.radians(.3):
                    break
                # 0.01rad/s is the smallest angular command OpenCR does not truncate.
                speed = .01 if abs(error) < math.radians(3) else self.cfg['max_angular_speed']
                self.command(0., math.copysign(speed, error))
            else:
                travelled = math.hypot(self.odometry[0]-start[0], self.odometry[1]-start[1])
                if travelled >= abs(value):
                    break
                # Blocked (e.g. a fork against the pallet, or collision_monitor): stop, don't push.
                if travelled > best+.001:
                    best, best_time = travelled, time.monotonic()
                elif time.monotonic()-max(best_time, self.link_resumed) > 3.:
                    raise RuntimeError(f'Drive stalled after {travelled*1000:.0f}mm of {abs(value)*1000:.0f}mm')
                self.command(math.copysign(linear, value), 0.)
        since = None
        while since is None or time.monotonic()-since < .5:
            if time.monotonic() > deadline:
                raise RuntimeError('Alignment timeout')
            self.command()
            rclpy.spin_once(self, timeout_sec=.02)
            since = (since or time.monotonic()) if self.stationary() else None
        # Frames exposed during motion must not enter the next observation.
        self.min_stamp = self.get_clock().now().nanoseconds

    def run(self):
        self.get_logger().info('EXECUTE: fixed-target alignment, insertion, lift, retreat' if self.execute else
                               'OBSERVE: no velocity commands')
        try:
            if self.execute:
                self.preflight_motion()
            deadline = time.monotonic()+self.cfg['alignment_timeout']
            for observation in range(1, self.cfg['max_observations']+1):
                pallet = self.observe()
                errors = alignment_errors(pallet, self.odometry, self.cfg)
                moves = [] if aligned(errors, self.cfg) else plan_moves(pallet, self.odometry, self.cfg)
                self.records.append({'t': time.monotonic()-self.start_time, 'observation': observation,
                                     'pallet_odom': pallet, 'odom': self.odometry,
                                     'errors': errors, 'moves': moves})
                gap, cross, heading, face = errors
                self.get_logger().info(f'Observation {observation}: gap={gap:.3f}m, face={face*1000:+.1f}mm, '
                                       f'heading={math.degrees(heading):+.2f}deg, centerline={cross:+.3f}m; '
                                       f'moves={moves}')
                if not moves:
                    self.get_logger().info('Alignment verified while stopped.')
                    if self.execute:
                        # Straight in on odometry; the camera cannot see the openings this close.
                        travel = gap-self.cfg['fork_projection']+self.cfg['insertion_depth']
                        # Kept for place_pallet; map (AMCL), since odom drifts over later trips.
                        self.placement = {'pose': self.map_pose(), 'travel': travel}
                        self.get_logger().info(f'Inserting forks: straight {travel*1000:.0f}mm forward')
                        self.records.append({'t': time.monotonic()-self.start_time, 'insertion_travel': travel,
                                             'placement': self.placement})
                        speed = self.cfg['insertion_speed']
                        self.lift_stopped('DOWN', INSERT_LIFT_DOWN)
                        self.move('drive', travel, time.monotonic()+travel/speed+30, speed)
                        self.lift_stopped('UP', self.cfg['lift_time'])
                        retreat = self.cfg['retreat_distance']
                        self.get_logger().info(f'Retreating straight {retreat*1000:.0f}mm')
                        self.move('drive', -retreat, time.monotonic()+retreat/speed+30, speed)
                    return True
                if gap < self.cfg['min_body_gap']:
                    raise RuntimeError('Pallet is closer than min_body_gap')
                if any(kind == 'drive' and abs(value) > self.cfg['max_move_distance'] for kind, value in moves):
                    raise RuntimeError('Planned move exceeds max_move_distance; check the observation')
                if not self.execute:
                    return False
                for kind, value in moves:
                    self.move(kind, value, deadline)
            raise RuntimeError('Alignment not verified within max_observations')
        finally:
            for _ in range(3):
                self.command()
                time.sleep(.03)

    def lift_stopped(self, command, seconds):
        lift(self, command, seconds, self.cancel)
        # Lifecycle polling pauses during the lift; refresh it before moving again.
        until = time.monotonic()+3
        while not self.chain_active():
            if time.monotonic() > until:
                raise RuntimeError('Velocity smoother/collision monitor state lost')
            rclpy.spin_once(self, timeout_sec=.05)

    def place(self, placement):
        """Correct onto the saved aligned map pose, then drive the saved insertion travel."""
        self.get_logger().info('EXECUTE: return to saved alignment pose, straight in to place')
        try:
            self.preflight_motion()
            deadline = time.monotonic()+self.cfg['alignment_timeout']
            tx, ty, tyaw = placement['pose']
            for correction in range(1, 4):
                x, y, yaw = self.map_pose()
                c, s = math.cos(yaw), math.sin(yaw)
                ahead, side = c*(tx-x)+s*(ty-y), -s*(tx-x)+c*(ty-y)
                heading = wrap_angle(tyaw-yaw)
                self.records.append({'t': time.monotonic()-self.start_time, 'correction': correction,
                                     'map_pose': (x, y, yaw), 'ahead': ahead, 'side': side, 'heading': heading})
                self.get_logger().info(f'Correction {correction}: ahead={ahead*1000:+.1f}mm, '
                                       f'side={side*1000:+.1f}mm, heading={math.degrees(heading):+.2f}deg')
                if (abs(side) <= self.cfg['lateral_tolerance']
                        and abs(heading) <= math.radians(self.cfg['yaw_tolerance_deg'])):
                    break
                distance = math.hypot(ahead, side)
                if distance > self.cfg['max_move_distance']:
                    raise RuntimeError('Saved pose is farther than max_move_distance; check AMCL')
                odom_yaw = self.odometry[2]
                if abs(side) > self.cfg['lateral_tolerance']:
                    # Turn at most 90deg toward (or away from) the saved point and drive straight onto it.
                    bearing = math.atan2(side, ahead)
                    if abs(bearing) > math.pi/2:
                        bearing, distance = wrap_angle(bearing+math.pi), -distance
                    self.move('turn', wrap_angle(odom_yaw+bearing), deadline)
                    self.move('drive', distance, deadline)
                self.move('turn', wrap_angle(odom_yaw+heading), deadline)
            else:
                raise RuntimeError('Saved pose not reached within 3 corrections')
            # Remaining along-heading offset is folded into the straight travel.
            travel = placement['travel']+ahead
            self.get_logger().info(f'Placing: straight {travel*1000:.0f}mm forward')
            self.records.append({'t': time.monotonic()-self.start_time, 'placement_travel': travel})
            speed = self.cfg['insertion_speed']
            self.move('drive', travel, time.monotonic()+travel/speed+30, speed)
            return True
        finally:
            for _ in range(3):
                self.command()
                time.sleep(.03)

    def finish(self, completed, reason, output):
        deadline = time.monotonic()+2
        while time.monotonic() < deadline and rclpy.ok():
            self.command()
            rclpy.spin_once(self, timeout_sec=.05)
        stopped = self.stationary()
        result = {'execute': self.execute, 'reason': reason,
                  'alignment_completed': completed, 'stopped_verified': stopped,
                  'first_motion_command_sent': self.first is not None,
                  'origin_odom': self.origin, 'last_odom': self.odometry,
                  'config': self.cfg, 'records': self.records}
        if self.origin is not None and self.odometry:
            result['net_odom_displacement_m'] = math.hypot(self.odometry[0]-self.origin[0], self.odometry[1]-self.origin[1])
            result['net_odom_yaw_deg'] = math.degrees(math.atan2(math.sin(self.odometry[2]-self.origin[2]), math.cos(self.odometry[2]-self.origin[2])))
        output = Path(output)
        output.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now(ZoneInfo('Asia/Seoul')).strftime('%Y%m%d_%H%M%S_%f')
        path = output / (stamp+('_execute' if self.execute else '_observe')+'.json')
        path.write_text(json.dumps(result, indent=2)+'\n')
        print(json.dumps({k: v for k, v in result.items() if k not in ('records', 'config')}, indent=2), flush=True)
        print(f'Trial report: {path}', flush=True)
        return stopped


def run_trial(cfg, execute=False, show=False, enable_camera=False, output=DEFAULT_OUTPUT, cancel=None,
              placement=None):
    """placement (dict), if given, receives the aligned map pose and insertion travel.

    enable_camera: on before alignment, off when it ends (success, failure or Ctrl+C);
    navigation in between needs no camera.
    """
    validate_trial_config(cfg)
    node = Trial(cfg, execute, show)
    node.cancel = cancel

    def work():
        if enable_camera:
            node.enable_camera()
        node.prepare_perception()
        completed = node.run()
        if placement is not None and node.placement:
            placement.update(node.placement)
        return completed
    try:
        return finish_run(node, work, execute, show, output)
    finally:
        if enable_camera:
            camera_off(cfg)


def place_pallet(cfg, placement, output=DEFAULT_OUTPUT, cancel=None):
    """Drive back onto a run_trial placement and straight in; the caller lowers the lift."""
    validate_trial_config(cfg)
    node = Trial(cfg, True)
    node.cancel = cancel
    return finish_run(node, lambda: node.place(placement), True, False, output)


def finish_run(node, work, execute, show, output):
    completed, reason, stopped = False, '', False
    try:
        completed = work()
    except (RuntimeError, KeyboardInterrupt) as exc:
        reason = str(exc) or 'User interrupted'
        raise
    finally:
        try:
            stopped = node.finish(completed, reason, output)
        finally:
            if show:
                cv2.destroyAllWindows()
            node.destroy_node()
    if execute and not stopped:
        raise RuntimeError('Trial ended but odometry did not confirm a stop')
    return completed


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=DEFAULT_CONFIG)
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--show', action='store_true')
    parser.add_argument('--enable-camera', action='store_true')
    parser.add_argument('--output', type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()
    try:
        cfg = load_config(args.config)
        validate_trial_config(cfg)
        if args.execute:
            validate_motion_config(cfg)
    except (ValueError, OSError) as exc:
        parser.error(str(exc))
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    try:
        return 0 if run_trial(cfg, args.execute, args.show, args.enable_camera, args.output) else 1
    except KeyboardInterrupt:
        return 130
    except RuntimeError as exc:
        print(f'Trial ended: {exc}')
        return 1
    finally:
        rclpy.try_shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
