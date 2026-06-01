#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import argparse
import math


def publish_cmd_pose(angle_deg: int):
    """
    发布 /cmd_pose 话题的控制指令。
    """

    # 创建一个发布者，发布到 /cmd_pose 话题
    cmd_pose_pub = rospy.Publisher('/cmd_pose', Twist, queue_size=10)

    # 创建Twist消息对象
    cmd_pose_msg = Twist()
    angle_rad = math.radians(angle_deg)
    # 设置位置指令
    cmd_pose_msg.linear.x = 0.0  # 基于当前位置的 x 方向值 (m)
    cmd_pose_msg.linear.y = 0.0  # 基于当前位置的 y 方向值 (m)
    cmd_pose_msg.linear.z = 0.0  # 增量高度 (m)
    cmd_pose_msg.angular.z = angle_rad*1.1  # 基于当前位置旋转（偏航）的角度，单位为弧度 (radian)

    # 未使用的字段设置为0
    cmd_pose_msg.angular.x = 0.0
    cmd_pose_msg.angular.y = 0.0

    # 确保发布者连接成功
    rate = rospy.Rate(10)  # 10Hz
    while cmd_pose_pub.get_num_connections() == 0 and not rospy.is_shutdown():
        rospy.loginfo("等待订阅者连接...")
        rate.sleep()

    # 发布消息
    cmd_pose_pub.publish(cmd_pose_msg)
    rospy.loginfo(f"Published cmd_pose: linear.x={cmd_pose_msg.linear.x}, linear.y={cmd_pose_msg.linear.y}, linear.z={cmd_pose_msg.linear.z}, angular.z={cmd_pose_msg.angular.z}")


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='踏步转向')
    parser.add_argument('--num', type=int, default=0,
                       help='转向角度')
    args = parser.parse_args()
    try:
        # 初始化ROS节点
        rospy.init_node('cmd_pose_publisher', anonymous=True)
        publish_cmd_pose(args.num)
    except rospy.ROSInterruptException:
        pass
