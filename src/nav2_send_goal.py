import math
import argparse
from pathlib import Path
import time

import rclpy
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

DESTINATIONS = {
    1: (0.531955480272539, -0.287590300840629, 270),
    2: None,
    3: None,
    4: None,
}

def navigate(nav, target, timeout):
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
    active = False
    try:
        if not nav.goToPose(goal_pose):
            print('목표가 거부되었습니다.')
            return False
        active = True
        deadline = time.monotonic() + timeout
        while not nav.isTaskComplete():
            if time.monotonic() > deadline:
                raise RuntimeError('Nav2 이동 시간 초과')
        active = False
        result = nav.getResult()
        print(f'이동 결과: {result.name}')
        return result == TaskResult.SUCCEEDED
    finally:
        if active and rclpy.ok():
            nav.cancelTask()


def main():
    parser = argparse.ArgumentParser(description='번호로 Nav2 목적지 선택')
    parser.add_argument(
        'destination',
        type=int,
        choices=DESTINATIONS.keys(),
        help='목적지 번호: 1, 2, 3, 4',
    )
    parser.add_argument('--align-pallet', action='store_true', help='목적지 이동 후 제한된 YOLO+LM 정렬 시험 (--execute 필요)')
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

    # Keep ROS alive during Ctrl+C cleanup so cancellation/zero commands can be sent.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    nav = None
    try:
        if not args.align_pallet or args.execute:
            nav = BasicNavigator()
            if not navigate(nav, target, args.nav_timeout):
                return 1  # Failed/canceled/rejected navigation never starts alignment/camera.
        if args.align_pallet:
            return 0 if run_trial(cfg, args.execute, args.show, args.enable_camera) else 1
        return 0
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
