import math
import argparse

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator

DESTINATIONS = {
    1: (0.531955480272539, -0.287590300840629, 270),
    2: None,
    3: None,
    4: None,
}
# X = 0.531955480272539
# Y = -0.287590300840629
# Z = 0.0
# YAW_DEG = 270.0
            

def main():
    parser = argparse.ArgumentParser(description='번호로 Nav2 목적지 선택')
    parser.add_argument(
        'destination',
        type=int,
        choices=DESTINATIONS.keys(),
        help='목적지 번호: 1, 2, 3, 4',
    )
    args = parser.parse_args()
    
    target = DESTINATIONS[args.destination]
    if target is None:
        parser.error(f'{args.destination}번 목적지 좌표가 설정되지 않았습니다.')

    x, y, yaw = target
    
    rclpy.init()
    nav = BasicNavigator()
    nav.waitUntilNav2Active()

    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = nav.get_clock().now().to_msg()
    goal_pose.pose.position.x = float(x)
    goal_pose.pose.position.y = float(y)
    goal_pose.pose.position.z = float(z)
    yaw = math.radians(yaw)
    goal_pose.pose.orientation.z = math.sin(yaw / 2.0)
    goal_pose.pose.orientation.w = math.cos(yaw / 2.0)

    if nav.goToPose(goal_pose):
        while not nav.isTaskComplete():
            pass  # 내부에서 ROS 응답을 최대 0.1초씩 기다립니다.
        print(f'이동 결과: {nav.getResult().name}')
    else:
        print('목표가 거부되었습니다.')

    nav.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()