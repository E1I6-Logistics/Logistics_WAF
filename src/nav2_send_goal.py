import math
import argparse
from pathlib import Path
import time

import cv2
import numpy as np
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from tf2_ros import Buffer, TransformListener

DESTINATIONS = {
    1: (0.512388313505402, -0.0015077706338736544, 270),
    2: (0.512388313505402, 0.5344194149087085, 90),
#    3: (0.762388313505402, 0.3044194149087085, 90),
    3: (1.052388313505402, 0.2444194149087085, 90),
    4: (1.905751886319870, -0.23540788720650954, 180)
}

# Scan-to-map correction before each Nav2 move (replaces RViz 2D Pose Estimate).
LOCALIZE_SEARCH = (0.30, math.radians(30))  # +-position (m), +-heading searched around AMCL's pose
LOCALIZE_MIN_SCORE = 0.5  # mean scan-to-wall match (0..1) required to move
NAV_RETRIES = 3  # retries after a Nav2 FAILED result or a sensor/TF dropout
BACKUP_DISTANCE = 0.15
NAV_LIFT_UP = 7.0  # s of lift UP while stopped before each Nav2 move


class LinkTimeout(RuntimeError):
    """Sensor/TF data did not arrive in time (Wi-Fi/DDS dropout); safe to retry."""


class AmclTimeout(LinkTimeout):
    """AMCL took the reset pose but processed no scan in time; the goal may still be sent."""


def yaw_of(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


def match_scan(grid, points, pose):
    """Best (score, x, y, yaw) for base-frame scan points on the map near pose, and pose's own score."""
    info, res = grid.info, grid.info.resolution
    cells = np.asarray(grid.data, np.int16).reshape(info.height, info.width)
    distance = cv2.distanceTransform(np.where(cells >= 65, 0, 255).astype(np.uint8), cv2.DIST_L2, 5)*res
    field = np.exp(-distance**2/(2*.05**2))
    px, py = points

    def score(x, y, yaw):
        c, s = np.cos(yaw)[..., None], np.sin(yaw)[..., None]
        col = np.floor((x[..., None]+c*px-s*py-info.origin.position.x)/res).astype(int)
        row = np.floor((y[..., None]+s*px+c*py-info.origin.position.y)/res).astype(int)
        inside = (col >= 0) & (col < info.width) & (row >= 0) & (row < info.height)
        return np.where(inside, field[row.clip(0, info.height-1), col.clip(0, info.width-1)], 0).mean(-1)

    best = (-1., *pose)
    for reach, angle, step, turn in ((LOCALIZE_SEARCH[0], LOCALIZE_SEARCH[1], .02, math.radians(1)),
                                     (.02, math.radians(1), .005, math.radians(.2))):
        _, bx, by, byaw = best
        offsets = np.arange(-reach, reach+step/2, step)
        x, y = np.meshgrid(bx+offsets, by+offsets)
        for yaw in byaw+np.arange(-angle, angle+turn/2, turn):
            values = score(x, y, np.full(x.shape, yaw))
            i = np.unravel_index(values.argmax(), values.shape)
            if values[i] > best[0]:
                best = (float(values[i]), float(x[i]), float(y[i]), float(yaw))
    return best, float(score(np.array(pose[0]), np.array(pose[1]), np.array(pose[2])))


class ExistingPoseNavigator(BasicNavigator):
    """Use AMCL's existing localization without republishing an initial pose."""

    def __init__(self):
        super().__init__()
        self.map_msg = self.scan_msg = self.amcl_stamp = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, 'map', lambda m: setattr(self, 'map_msg', m), latched)
        self.create_subscription(LaserScan, 'scan', lambda m: setattr(self, 'scan_msg', m), qos_profile_sensor_data)

    def localize(self):
        """Match the stopped robot's scan to the map around AMCL's pose; reset AMCL to the best fit."""
        self.scan_msg = None
        deadline = time.monotonic()+5
        while (self.map_msg is None or self.scan_msg is None
               or not self.tf_buffer.can_transform('map', 'base_footprint', Time())
               or not self.tf_buffer.can_transform('base_footprint', self.scan_msg.header.frame_id, Time())):
            if time.monotonic() > deadline:
                raise LinkTimeout('Localization needs /map, /scan and map->base_footprint TF')
            rclpy.spin_once(self, timeout_sec=.1)
        scan = self.scan_msg
        robot = self.tf_buffer.lookup_transform('map', 'base_footprint', Time()).transform
        sensor = self.tf_buffer.lookup_transform('base_footprint', scan.header.frame_id, Time()).transform
        ranges = np.asarray(scan.ranges, float)
        angles = scan.angle_min+np.arange(len(ranges))*scan.angle_increment+yaw_of(sensor.rotation)
        valid = np.isfinite(ranges) & (ranges > scan.range_min) & (ranges < scan.range_max)
        ranges, angles = ranges[valid], angles[valid]
        points = (sensor.translation.x+ranges*np.cos(angles), sensor.translation.y+ranges*np.sin(angles))
        pose = (robot.translation.x, robot.translation.y, yaw_of(robot.rotation))
        (best, x, y, yaw), current = match_scan(self.map_msg, points, pose)
        self.info(f'Localization: match {current:.2f} -> {best:.2f}, shift '
                  f'({x-pose[0]:+.3f}, {y-pose[1]:+.3f})m {math.degrees(yaw-pose[2]):+.1f}deg')
        if best < LOCALIZE_MIN_SCORE:
            raise RuntimeError(f'위치 추정 실패: 스캔-지도 일치 {best:.2f} < {LOCALIZE_MIN_SCORE}. '
                               '로봇을 초기 위치 근처에 두거나 RViz 2D Pose Estimate로 설정하세요.')
        message = PoseWithCovarianceStamped()
        message.header.frame_id = 'map'
        message.header.stamp = self.get_clock().now().to_msg()
        message.pose.pose.position.x, message.pose.pose.position.y = x, y
        message.pose.pose.orientation.z, message.pose.pose.orientation.w = math.sin(yaw/2), math.cos(yaw/2)
        message.pose.covariance[0] = message.pose.covariance[7] = .05**2
        message.pose.covariance[35] = math.radians(3)**2
        self.amcl_stamp = None
        self.initial_pose_pub.publish(message)
        # Plan only once AMCL has processed a scan since the reset and map->odom is fresh;
        # a /scan dropout here otherwise aborts the controller on a stale map->odom.
        reset = Time.from_msg(message.header.stamp)
        deadline = time.monotonic()+5
        while True:
            rclpy.spin_once(self, timeout_sec=.1)
            if self.amcl_stamp is not None and self.amcl_stamp > reset:
                stamp = self.tf_buffer.lookup_transform('map', 'odom', Time()).header.stamp
                if (self.get_clock().now()-Time.from_msg(stamp)).nanoseconds < 2e8:
                    return
            if time.monotonic() > deadline:
                raise AmclTimeout('AMCL did not refresh map->odom within 5s of the reset (check /scan)')

    def _amclPoseCallback(self, msg):
        self.amcl_stamp = Time.from_msg(msg.header.stamp)
        super()._amclPoseCallback(msg)

    def _waitForInitialPose(self):
        # Jazzy BasicNavigator publishes its default pose before spinning here.
        deadline = time.monotonic() + 30.0
        if not self.initial_pose_received:
            self.info('Waiting for existing AMCL pose; initial pose will not be reset')
        while not self.initial_pose_received:
            if not rclpy.ok():
                raise RuntimeError('ROS stopped while waiting for AMCL pose')
            if time.monotonic() >= deadline:
                raise RuntimeError(
                    'AMCL 위치를 30초 동안 수신하지 못했습니다. '
                    'RViz의 2D Pose Estimate로 실제 위치·방향을 설정하고 다시 실행하세요.')
            rclpy.spin_once(self, timeout_sec=0.1)


def navigate(nav, target, timeout, cancel=None):
    from pallet_alignment_trial import lift
    x, y, yaw = target
    nav.waitUntilNav2Active()
    lift(nav, 'UP', NAV_LIFT_UP, cancel)
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.pose.position.x = float(x)
    goal_pose.pose.position.y = float(y)
    goal_pose.pose.position.z = 0.0
    yaw = math.radians(yaw)
    goal_pose.pose.orientation.z = math.sin(yaw / 2.0)
    goal_pose.pose.orientation.w = math.cos(yaw / 2.0)
    # Retries: a brief /scan or TF dropout makes Nav2 fail at once on a stale transform.
    for attempt in range(NAV_RETRIES+1):
        try:
            nav.localize()
        except LinkTimeout as error:
            if attempt < NAV_RETRIES:
                print(f'{error}: 재시도합니다 ({attempt+1}/{NAV_RETRIES}).')
                continue
            if not isinstance(error, AmclTimeout):
                raise
            print(f'경고: {error}; 위치 보정 확인 없이 이동합니다.')
        goal_pose.header.stamp = nav.get_clock().now().to_msg()
        if not nav.goToPose(goal_pose):
            print('목표가 거부되었습니다.')
            return False
        if wait_task(nav, timeout, '이동', cancel):
            return True
        if attempt == NAV_RETRIES or nav.getResult() != TaskResult.FAILED:
            return False
        print(f'이동 실패: 위치를 다시 맞추고 재시도합니다 ({attempt+1}/{NAV_RETRIES}).')


def wait_task(nav, timeout, name, cancel=None):
    active = True
    try:
        deadline = time.monotonic() + timeout
        while not nav.isTaskComplete():
            if time.monotonic() > deadline:
                raise RuntimeError(f'Nav2 {name} 시간 초과')
            if cancel and cancel():
                raise RuntimeError('Canceled')
        active = False
        result = nav.getResult()
        print(f'{name} 결과: {result.name}')
        return result == TaskResult.SUCCEEDED
    finally:
        if active and rclpy.ok():
            nav.cancelTask()


def lower(nav, cfg, suffix='', stage=lambda name: None, cancel=None):
    """Lift down where the robot stands, then back the forks out."""
    from pallet_alignment_trial import lift
    stage('LIFT_DOWN'+suffix)
    lift(nav, 'DOWN', cfg['lift_time'], cancel)
    # Nav2 BackUp behavior (cmd_vel_nav chain, costmap collision check).
    stage('BACKUP'+suffix)
    # Odom (not map) measures what was already backed, so a retry covers only the rest.
    start = odom_xy(nav)
    for attempt in range(NAV_RETRIES+1):
        x, y = odom_xy(nav)
        remaining = BACKUP_DISTANCE-math.hypot(x-start[0], y-start[1])
        if remaining < .01:
            return True
        if not nav.backup(backup_dist=remaining, backup_speed=0.025, time_allowance=20):
            return False
        if wait_task(nav, 30, '후진', cancel):
            return True
        if attempt == NAV_RETRIES or nav.getResult() != TaskResult.FAILED:
            return False
        print(f'후진 실패: 남은 거리를 재시도합니다 ({attempt+1}/{NAV_RETRIES}).')
    return False


def odom_xy(nav):
    for _ in range(3):  # Pick up the latest odom TF before reading it.
        rclpy.spin_once(nav, timeout_sec=.05)
    t = nav.tf_buffer.lookup_transform('odom', 'base_footprint', Time()).transform.translation
    return t.x, t.y


def replace_pallet(nav, target, cfg, timeout, show=False, enable_camera=False, stage=lambda name: None, cancel=None):
    """Pick at target → drop at destination 2 → pick at destination 3 → place at target's spot → destination 4."""
    from pallet_alignment_trial import place_pallet, run_trial
    placement = {}
    stage('NAV_TO_PICK')
    if not navigate(nav, target, timeout, cancel):
        return False  # Failed/canceled/rejected navigation never starts alignment/camera.
    stage('ALIGN')
    if not run_trial(cfg, True, show, enable_camera, cancel=cancel, placement=placement):
        return False
    # Carry the lifted pallet to destination 2 and lower it there.
    stage('NAV_TO_DROP')
    if not navigate(nav, DESTINATIONS[2], timeout, cancel):
        return False
    if not lower(nav, cfg, '', stage, cancel):
        return False
    # Pick the pallet in front of destination 3 the same way.
    stage('NAV_TO_PICK_2')
    if not navigate(nav, DESTINATIONS[3], timeout, cancel):
        return False
    stage('ALIGN_2')
    if not run_trial(cfg, True, show, enable_camera, cancel=cancel):
        return False
    # Put it where the first pallet was: saved aligned map pose, then the same straight travel.
    stage('NAV_TO_PLACE')
    x, y, yaw = placement['pose']
    if not navigate(nav, (x, y, math.degrees(yaw)), timeout, cancel):
        return False
    stage('PLACE')
    if not place_pallet(cfg, placement, cancel=cancel):
        return False
    if not lower(nav, cfg, '_2', stage, cancel):
        return False
    stage('NAV_TO_HOME')
    return navigate(nav, DESTINATIONS[4], timeout, cancel)


def main():
    parser = argparse.ArgumentParser(description='번호로 Nav2 목적지 선택')
    parser.add_argument(
        'destination',
        type=int,
        choices=DESTINATIONS.keys(),
        help='목적지 번호: 1, 2, 3, 4',
    )
    parser.add_argument('--align-pallet', action='store_true', help='팔레트 정렬. --execute 시 목적지 이동 → 정렬·삽입·리프트업·후진 → 목적지 2 이동 → 리프트다운 → 15cm 후진 → 목적지 3에서 같은 방식으로 회수 → 첫 정렬 위치로 이동·직진 → 리프트다운 → 15cm 후진 → 목적지 4 이동, 없으면 현재 위치에서 관찰만')
    parser.add_argument('--execute', action='store_true', help='팔레트 테스트의 실제 이동 허용')
    parser.add_argument('--show', action='store_true', help='입구 검출 영상 표시')
    parser.add_argument('--enable-camera', action='store_true', help='도착 후 Pi 카메라 서비스 호출')
    parser.add_argument('--alignment-config', type=Path,
                        default=Path(__file__).resolve().parents[1]/'config/trial.yaml',
                        help='정렬 시험 설정 (기본: config/trial.yaml)')
    parser.add_argument('--nav-timeout', type=float, default=600.0)
    args = parser.parse_args()
    if (args.execute or args.show or args.enable_camera) and not args.align_pallet:
        parser.error('--execute/--show/--enable-camera는 --align-pallet과 함께 사용합니다.')
    if not math.isfinite(args.nav_timeout) or args.nav_timeout <= 0:
        parser.error('--nav-timeout must be positive and finite')
    target = DESTINATIONS[args.destination]
    if target is None:
        parser.error(f'{args.destination}번 목적지 좌표가 설정되지 않았습니다.')

    cfg = None
    if args.align_pallet:
        # Import/validate before a navigation goal can move the robot.
        from pallet_alignment_trial import run_trial, validate_trial_config
        from pallet_vision import load_config, validate_motion_config
        try:
            cfg = load_config(args.alignment_config)
            validate_trial_config(cfg)
            if args.execute:
                validate_motion_config(cfg)
        except (ValueError, OSError) as error:
            parser.error(str(error))
        if not args.execute:
            print(f'관찰 모드: 목적지 {args.destination} {target}; Nav2 목표와 속도 명령을 보내지 않습니다.')
        else:
            print(f'목적지 {args.destination} {target}의 팔레트를 목적지 2 {DESTINATIONS[2]}에 내려놓고, 목적지 3 {DESTINATIONS[3]}의 팔레트를 그 자리에 옮긴 뒤 목적지 4 {DESTINATIONS[4]}로 이동합니다.')

    # Keep ROS alive during Ctrl+C cleanup so cancellation/zero commands can be sent.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    nav = None
    try:
        # Observe mode (--align-pallet without --execute) never sends a Nav2 goal.
        if not args.align_pallet:
            nav = ExistingPoseNavigator()
            return 0 if navigate(nav, target, args.nav_timeout) else 1
        if not args.execute:
            return 0 if run_trial(cfg, False, args.show, args.enable_camera) else 1
        nav = ExistingPoseNavigator()
        return 0 if replace_pallet(nav, target, cfg, args.nav_timeout, args.show, args.enable_camera) else 1
    except KeyboardInterrupt:
        print('사용자가 중단했습니다.')
        return 130
    except RuntimeError as error:
        print(f'작업 중단: {error}')
        return 1
    finally:
        if nav is not None:
            nav.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
