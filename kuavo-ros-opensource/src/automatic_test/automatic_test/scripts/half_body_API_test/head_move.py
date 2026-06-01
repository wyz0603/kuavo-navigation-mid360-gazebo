#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from kuavo_msgs.msg import robotHeadMotionData
from std_msgs.msg import Bool

def set_head_target(yaw, pitch):
    """
    设置头部目标位置，并发布消息
    :param yaw: 头部的偏航角，范围为[-30, 30]度
    :param pitch: 头部的俯仰角，范围为[-25, 25]度
    """
    
    # 创建一个发布者，发布到'/robot_head_motion_data'话题
    # 使用robotHeadMotionData消息类型，队列大小为10
    pub_head_pose = rospy.Publisher('/robot_head_motion_data', robotHeadMotionData, queue_size=10)
    rospy.sleep(0.5)  # 确保Publisher注册
    function_status_pub = rospy.Publisher('funtion_finish', Bool, queue_size=1)
    # 创建一个robotHeadMotionData消息对象
    head_target_msg = robotHeadMotionData()
    
    # 设置关节数据，包含偏航和俯仰角
    # 确保yaw在[-30, 30]范围内，pitch在[-25, 25]范围内
    head_target_msg.joint_data = [yaw, pitch]
    
    # 等待订阅者连接
    rate = rospy.Rate(10)  # 10Hz
    while pub_head_pose.get_num_connections() == 0 and not rospy.is_shutdown():
        rospy.loginfo("等待订阅者连接...")
        rate.sleep()

    # 发布消息到指定话题
    pub_head_pose.publish(head_target_msg)
    
    # 打印日志信息，显示已发布的头部目标位置
    rospy.loginfo(f"Published head target: yaw={yaw}, pitch={pitch}")
    function_status_pub.publish(True)

if __name__ == '__main__':
    try:
        # 初始化ROS节点，节点名称为'robot_head_controller'
        rospy.init_node('robot_head_controller', anonymous=True)
        # 发布一次头部目标位置为0, 0
        set_head_target(10, 20)
    except rospy.ROSInterruptException:
        # 捕获ROS中断异常，通常在节点关闭时发生
        pass