import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator

X = 1.0
Y = 0.0
YAW_DEG = 0.0


def main():
    rclpy.init()
    nav = BasicNavigator()
    nav.waitUntilNav2Active()

    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = nav.get_clock().now().to_msg()
    goal_pose.pose.position.x = float(X)
    goal_pose.pose.position.y = float(Y)
    yaw = math.radians(YAW_DEG)
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