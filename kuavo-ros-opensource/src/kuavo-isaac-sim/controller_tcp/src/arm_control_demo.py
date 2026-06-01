#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState

def publish_arm_positions():
    # 初始化ROS节点
    rospy.init_node('arm_position_publisher', anonymous=True)
    
    # 创建发布者
    pub = rospy.Publisher('/kuavo_arm_traj', JointState, queue_size=10)
    
    # 设置发布频率
    rate = rospy.Rate(10)  # 10Hz
    
    # 创建JointState消息
    joint_state_msg = JointState()
    
    # 设置关节名称（按照指定顺序）
    joint_state_msg.name = [
        'zarm_l1_joint',
        'zarm_l2_joint',
        'zarm_l3_joint',
        'zarm_l4_joint',
        'zarm_l5_joint',
        'zarm_l6_joint',
        'zarm_l7_joint',
        'zarm_r1_joint',
        'zarm_r2_joint',
        'zarm_r3_joint',
        'zarm_r4_joint',
        'zarm_r5_joint',
        'zarm_r6_joint',
        'zarm_r7_joint'
    ]
    
    # 设置目标位置（单位：弧度）
    # 这里设置一些示例位置，你可以根据需要修改这些值
    joint_state_msg.position = [
        -50.0,    # zarm_l1_joint
        0.0,    # zarm_l2_joint
        0.0,    # zarm_l3_joint
        -30.0,    # zarm_l4_joint
        0.0,    # zarm_l5_joint
        0.0,    # zarm_l6_joint
        0.0,    # zarm_l7_joint
        -50.0,    # zarm_r1_joint
        0.0,   # zarm_r2_joint
        0.0,    # zarm_r3_joint
        -30.0,   # zarm_r4_joint
        0.0,    # zarm_r5_joint
        0.0,   # zarm_r6_joint
        0.0     # zarm_r7_joint
    ]

    # joint_state_msg.position = [
    #     0.0,    # zarm_l1_joint
    #     0.0,    # zarm_l2_joint
    #     0.0,    # zarm_l3_joint
    #     0.0,    # zarm_l4_joint
    #     0.0,    # zarm_l5_joint
    #     0.0,    # zarm_l6_joint
    #     0.0,    # zarm_l7_joint
    #     0.0,    # zarm_r1_joint
    #     0.0,   # zarm_r2_joint
    #     0.0,    # zarm_r3_joint
    #     0.0,   # zarm_r4_joint
    #     0.0,    # zarm_r5_joint
    #     0.0,   # zarm_r6_joint
    #     0.0     # zarm_r7_joint
    # ]
    
    while not rospy.is_shutdown():
        # 更新时间戳
        joint_state_msg.header.stamp = rospy.Time.now()
        
        # 发布消息
        pub.publish(joint_state_msg)
        
        # 按照设定的频率休眠
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_arm_positions()
    except rospy.ROSInterruptException:
        pass
