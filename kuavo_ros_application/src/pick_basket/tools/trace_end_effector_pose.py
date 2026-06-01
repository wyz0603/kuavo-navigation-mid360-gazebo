#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import tf2_ros
from geometry_msgs.msg import Point
import visualization_msgs.msg
import std_msgs.msg

def publish_trajectory(target_frame, source_frame):
    # 初始化ROS节点
    rospy.init_node('trajectory_publisher')

    # 创建tf2缓冲区
    tf_buffer = tf2_ros.Buffer()
    # 创建一个tf2监听器，绑定缓冲区
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    # 创建一个Publisher，发布轨迹数据
    trajectory_publisher = rospy.Publisher('/eef_trajectory', visualization_msgs.msg.Marker, queue_size=10)

    rate = rospy.Rate(10.0)  # 每秒检查10次变换

    points = visualization_msgs.msg.Marker()
    points.header.frame_id = target_frame
    points.ns = "trajectory"
    points.id = 0
    points.type = visualization_msgs.msg.Marker.LINE_STRIP
    points.action = visualization_msgs.msg.Marker.ADD
    points.pose.orientation.w = 1.0  # 设置四元数
    points.scale.x = 0.01  # 线条宽度
    points.color.a = 1.0  # 不透明度
    points.color.r = 0.0  # 红色
    points.color.g = 0.8  # 绿色
    points.color.b = 0.0  # 蓝色

    # 开始监听变换并记录轨迹
    while not rospy.is_shutdown():
        try:
            # 监听从source_frame到target_frame的变换
            trans = tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                rospy.Time(0),
                rospy.Duration(1.0)  # 超时时间设置为1秒
            )

            # 将新的位置添加到轨迹中
            new_point = Point()
            new_point.x = trans.transform.translation.x
            new_point.y = trans.transform.translation.y
            new_point.z = trans.transform.translation.z
            points.points.append(new_point)

            # 发布轨迹数据
            points.header.stamp = rospy.Time.now()
            trajectory_publisher.publish(points)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            print("Failed to get transform: {}".format(e))
            continue

        rate.sleep()

if __name__ == '__main__':
    try:
        publish_trajectory('base_link', 'zarm_r7_end_effector')
    except rospy.ROSInterruptException:
        pass