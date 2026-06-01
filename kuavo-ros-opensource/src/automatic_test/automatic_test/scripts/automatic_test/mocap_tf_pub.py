#!/usr/bin/env python3

import rospy
import tf2_ros
import geometry_msgs.msg
from geometry_msgs.msg import Pose

# 全局变量
br = None

def pose_callback(msg):
    # 创建 TransformStamped 消息
    t = geometry_msgs.msg.TransformStamped()
    
    # 设置时间戳
    t.header.stamp = rospy.Time.now()
    
    # 设置坐标系
    t.header.frame_id = "mocap_frame"
    t.child_frame_id = "mocap_robot"
    
    # 设置平移
    t.transform.translation.x = msg.position.x / 1000
    t.transform.translation.y = msg.position.y / 1000
    t.transform.translation.z = msg.position.z / 1000
    
    # 设置旋转
    t.transform.rotation.x = msg.orientation.x
    t.transform.rotation.y = msg.orientation.y
    t.transform.rotation.z = msg.orientation.z
    t.transform.rotation.w = msg.orientation.w
    
    # 发布变换
    if br is not None:
        br.sendTransform(t)

def main():
    global br
    
    # 初始化 ROS 节点
    rospy.init_node('mocap_tf_publisher')
    
    # 创建 TF 广播器
    br = tf2_ros.TransformBroadcaster()
    
    # 订阅 /robot_pose 话题
    rospy.Subscriber('/robot_pose', Pose, pose_callback)
    
    # 保持节点运行
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
