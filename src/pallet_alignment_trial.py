"""Reusable bounded alignment trial. Observe mode never publishes velocity."""
import argparse
from datetime import datetime
import json
import math
from pathlib import Path
import time
from zoneinfo import ZoneInfo

import rclpy
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped
from action_msgs.msg import GoalStatusArray
from turtlebot3_msgs.msg import SensorState

from pallet_alignment import PalletAligner
from pallet_vision import load_config, validate_motion_config

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONFIG = ROOT / 'config/trial.yaml'
DEFAULT_OUTPUT = ROOT / 'output/pallet_alignment_trials'


def validate_trial_config(cfg):
    """Do not let a config override silently remove bounded-trial limits."""
    if cfg.get('detector') != 'yolo_pose' or cfg.get('pose_refine_lm') is not True:
        raise ValueError('Trial requires detector: yolo_pose and pose_refine_lm: true')
    limits = {'trial_max_motion_seconds': 3.0, 'trial_max_displacement': .035,
              'trial_max_yaw_change': .16, 'max_linear_speed': .010,
              'max_angular_speed': .050}
    for key, ceiling in limits.items():
        value = cfg.get(key)
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or not 0 < value <= ceiling:
            raise ValueError(f'{key} must be positive and <= {ceiling}')


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

    def motion_guard(self):
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
        if self.first is not None and now-self.first >= self.cfg['trial_max_motion_seconds']:
            raise RuntimeError('Bounded trial motion time limit reached')
        if self.origin is not None:
            distance = math.hypot(self.odometry[0]-self.origin[0], self.odometry[1]-self.origin[1])
            yaw = abs(math.atan2(math.sin(self.odometry[2]-self.origin[2]),
                                 math.cos(self.odometry[2]-self.origin[2])))
            if distance >= self.cfg['trial_max_displacement'] or yaw >= self.cfg['trial_max_yaw_change']:
                raise RuntimeError('Odometry travel cap reached')

    def preflight_motion(self):
        until = time.monotonic()+3
        while time.monotonic() < until:
            rclpy.spin_once(self, timeout_sec=.05)
        self.motion_guard()
        if abs(self.odometry[4]) > .005 or abs(self.odometry[5]) > .02:
            raise RuntimeError('Robot is already moving')
        super().preflight_motion()

    def chain_active(self):
        # Check bounds even when detection is lost and the next command would be zero.
        if self.execute:
            self.motion_guard()
        return super().chain_active()

    def command(self, linear=0., angular=0.):
        if self.execute and (linear or angular):
            try:
                self.motion_guard()
                if (not math.isfinite(linear) or not math.isfinite(angular) or linear < 0
                        or linear > self.cfg['max_linear_speed']+1e-9
                        or abs(angular) > self.cfg['max_angular_speed']+1e-9):
                    raise RuntimeError('Command exceeds trial bounds')
            except RuntimeError:
                super().command()
                raise
            if self.first is None:
                self.first, self.origin = time.monotonic(), self.odometry[:3]
        self.records.append({'t': time.monotonic()-self.start_time,
                             'v': linear, 'w': angular, 'odom': self.odometry})
        super().command(linear, angular)

    def base_target(self, detection, message):
        target = super().base_target(detection, message)
        self.records.append({'t': time.monotonic()-self.start_time,
                             'target': target, 'rms_px': detection.error})
        return target

    def finish(self, completed, reason, output):
        deadline = time.monotonic()+2
        while time.monotonic() < deadline and rclpy.ok():
            self.command()
            rclpy.spin_once(self, timeout_sec=.05)
        stopped = bool(self.odometry and all(math.isfinite(v) for v in self.odometry)
                       and time.monotonic()-self.odometry[3] < .5
                       and abs(self.odometry[4]) < .005 and abs(self.odometry[5]) < .02)
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


def run_trial(cfg, execute=False, show=False, enable_camera=False, output=DEFAULT_OUTPUT):
    validate_trial_config(cfg)
    node = Trial(cfg, execute, show)
    completed, reason, stopped = False, '', False
    try:
        if enable_camera:
            node.enable_camera()
        completed = node.run()
    except (RuntimeError, KeyboardInterrupt) as exc:
        reason = str(exc) or 'User interrupted'
        raise
    finally:
        try:
            stopped = node.finish(completed, reason, output)
        finally:
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
