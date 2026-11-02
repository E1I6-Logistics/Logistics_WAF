import math
import argparse
from pathlib import Path
import time

import rclpy
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

DESTINATIONS = {
    1: (0.512388313505402, -0.0015077706338736544, 270),
    2: (0.512388313505402, 0.5344194149087085, 90),
#    3: (0.762388313505402, 0.3044194149087085, 90),
    3: (1.052388313505402, 0.2444194149087085, 90),
    4: (1.905751886319870, -0.23540788720650954, 180)
}

class ExistingPoseNavigator(BasicNavigator):
    """Use AMCL's existing localization without republishing an initial pose."""

    def _waitForInitialPose(self):
        # Jazzy BasicNavigator publishes its default pose before spinning here.
        # A subscriber discovery delay must not reset AMCL's existing estimate.
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
    x, y, yaw = target
    nav.waitUntilNav2Active()
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = nav.get_clock().now().to_msg()
    goal_pose.pose.position.x = float(x)
    goal_pose.pose.position.y = float(y)
    goal_pose.pose.position.z = 0.0
    yaw = math.radians(yaw)
    goal_pose.pose.orientation.z = math.sin(yaw / 2.0)
    goal_pose.pose.orientation.w = math.cos(yaw / 2.0)
    if not nav.goToPose(goal_pose):
        print('목표가 거부되었습니다.')
        return False
    return wait_task(nav, timeout, '이동', cancel)


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
    if not nav.backup(backup_dist=0.15, backup_speed=0.025, time_allowance=20):
        return False
    return wait_task(nav, 30, '후진', cancel)


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
