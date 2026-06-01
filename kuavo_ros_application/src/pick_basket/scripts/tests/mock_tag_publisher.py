#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import tf2_ros
from apriltag_ros.msg import AprilTagDetection, AprilTagDetectionArray
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, Point, Quaternion, TransformStamped
from tf.transformations import quaternion_from_euler, quaternion_multiply, quaternion_inverse
from tf2_ros import TransformBroadcaster

import math

# 定义标签ID、大小和位置
basket_tag_id = 0
desktop_tag_id = 7
basket_tag_size = 0.05
desktop_tag_size = 0.05
basket_tag_position = Point(1.2801, 0.50, 0.77)
desktop_tag_position = Point(1.2801, 0.50, 0.77)

# Convert 60 degrees to radians
angle_rad = math.radians(30)

# Create quaternions for 60 degree rotation around Z-axis
basket_tag_orientation = Quaternion(*quaternion_from_euler(0, math.radians(-90), angle_rad))
desktop_tag_orientation = Quaternion(*quaternion_from_euler(0, math.radians(-90), angle_rad))

def create_mock_detection(tf_buffer):
    # 创建篮子标签的检测数据
    basket_tag = AprilTagDetection()
    basket_tag.id.append(basket_tag_id)
    basket_tag.size = [basket_tag_size]
    basket_pose = PoseWithCovarianceStamped()
    basket_pose.header.stamp = rospy.Time.now()
    basket_pose.header.frame_id = "base_link"  # 修改 frame_id 为 base_link
    
    # 创建桌面标签的检测数据
    desktop_tag = AprilTagDetection()
    desktop_tag.id.append(desktop_tag_id)
    desktop_tag.size = [desktop_tag_size]
    desktop_pose = PoseWithCovarianceStamped()
    desktop_pose.header.stamp = rospy.Time.now()
    desktop_pose.header.frame_id = "base_link"  # 修改 frame_id 为 base_link

    # 获取 odom 到 base_link 的变换
    try:
        transform = tf_buffer.lookup_transform("odom", "base_link", rospy.Time(0))
        
        # 转换篮子标签的位置和方向
        basket_pose.pose.pose.position, basket_pose.pose.pose.orientation = transform_pose(
            basket_tag_position, basket_tag_orientation, transform)
        basket_tag.pose = basket_pose

        # 转换桌面标签的位置和方向
        desktop_pose.pose.pose.position, desktop_pose.pose.pose.orientation = transform_pose(
            desktop_tag_position, desktop_tag_orientation, transform)
        desktop_tag.pose = desktop_pose

    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        rospy.logwarn(f"TF lookup failed: {e}")
        return None

    # 构造消息
    msg = AprilTagDetectionArray()
    msg.detections.append(basket_tag)
    msg.detections.append(desktop_tag)
    return msg

def transform_pose(position, orientation, transform):
    # 转换位置
    transformed_position = Point()
    transformed_position.x = position.x - transform.transform.translation.x
    transformed_position.y = position.y - transform.transform.translation.y
    transformed_position.z = position.z - transform.transform.translation.z

    # 转换方向
    q1 = [orientation.x, orientation.y, orientation.z, orientation.w]
    q2 = [transform.transform.rotation.x, transform.transform.rotation.y,
          transform.transform.rotation.z, transform.transform.rotation.w]
    q2_inv = quaternion_inverse(q2)
    q_new = quaternion_multiply(q2_inv, q1)
    
    transformed_orientation = Quaternion(*q_new)

    return transformed_position, transformed_orientation

def broadcast_transforms():
    br = TransformBroadcaster()
    t = TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "odom"  # 设置父坐标系
    t.child_frame_id = f"tag_{desktop_tag_id}"  # 子坐标系为 tag_加上id
    t.transform.translation.x = desktop_tag_position.x
    t.transform.translation.y = desktop_tag_position.y
    t.transform.translation.z = desktop_tag_position.z
    t.transform.rotation = desktop_tag_orientation
    br.sendTransform(t)

def mock_publisher():
    # 初始化ROS节点
    rospy.init_node('mock_apriltag_publisher', anonymous=True)
    pub = rospy.Publisher('/robot_tag_info', AprilTagDetectionArray, queue_size=10)
    
    # 创建 TF 缓冲区和监听器
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    
    rate = rospy.Rate(10)  # 设置循环频率为10Hz
    while not rospy.is_shutdown():
        broadcast_transforms()  # 广播变换           
        msg = create_mock_detection(tf_buffer)  # 每次循环创建新的消息以更新时间戳
        if msg:
            pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        mock_publisher()
    except rospy.ROSInterruptException:
        pass