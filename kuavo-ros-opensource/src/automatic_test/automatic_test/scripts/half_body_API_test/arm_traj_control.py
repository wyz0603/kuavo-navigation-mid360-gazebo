#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import JointState
import numpy as np
import time
from kuavo_msgs.srv import changeArmCtrlMode, changeArmCtrlModeRequest
from std_msgs.msg import Bool
def publish_joint_states(q_now):
    """
    发布当前关节状态到指定话题。
    :param q_now: 当前关节位置列表
    """
    # 创建一个发布者，发布到 "/kuavo_arm_traj" 话题，消息类型为 JointState
    pub = rospy.Publisher("/kuavo_arm_traj", JointState, queue_size=10)
    rospy.sleep(0.5)

    msg = JointState()
    # 设置关节名称，假设有14个关节，名称为 "arm_joint_1" 到 "arm_joint_14"
    msg.name = ["arm_joint_" + str(i) for i in range(1, 15)]
    # 设置消息的时间戳为当前时间
    msg.header.stamp = rospy.Time.now()
    # 设置关节位置
    msg.position = np.array(q_now)
    # 确保关节位置数组长度为14
    assert len(msg.position) == 14, f"Position array length is {len(msg.position)}, expected 14"
    # 发布消息
    pub.publish(msg)

def interpolate_joint_positions(q0, q1, num):
    """
    生成从 q0 到 q1 的插值轨迹。
    :param q0: 初始关节位置列表
    :param q1: 目标关节位置列表
    :param num: 插值步数
    :return: 插值后的关节位置序列
    """
    q_list = []
    for i in range(num):
        q_tmp = [0.0] * 14  # 临时存储每一步的关节位置
        for j in range(14):
            # 线性插值计算每个关节的位置
            q_tmp[j] = q0[j] + i / float(num) * (q1[j] - q0[j])
        q_list.append(q_tmp)  # 将计算出的关节位置添加到列表中
    return q_list

# 设置手臂运动模式
def set_arm_control_mode(mode):
    # 创建服务代理，用于与服务通信
    arm_traj_change_mode_client = rospy.ServiceProxy("/arm_traj_change_mode", changeArmCtrlMode)

    # 创建请求对象
    request = changeArmCtrlModeRequest()
    request.control_mode = mode  # 设置请求的控制模式

    # 发送请求并接收响应
    response = arm_traj_change_mode_client(request)

    if response.result:
        # 如果响应结果为真，表示成功更改控制模式
        rospy.loginfo(f"Successfully changed arm control mode to {mode}: {response.message}")
    else:
        # 如果响应结果为假，表示更改控制模式失败
        rospy.logwarn(f"Failed to change arm control mode to {mode}: {response.message}")

def main():
    # 初始化ROS节点
    rospy.init_node('sim_traj', anonymous=True)
    function_status_pub = rospy.Publisher('funtion_finish', Bool, queue_size=1)
    rate = rospy.Rate(10)  # 设置发布频率为10Hz

    set_arm_control_mode(2)

    q0 = [0.0] * 14  # 初始关节位置，14个关节均为0
    q1 = [0.0] * 14  # 目标关节位置
    q1[:7] = [-30, 60, 0, -30, 0, -30, 30]  # 设置前7个关节的目标位置
    num = 90  # 插值的步数

    # 调用插值函数生成关节位置序列
    q_list = interpolate_joint_positions(q0, q1, num)

    # 发布插值后的关节位置序列
    for q in q_list:
        publish_joint_states(q)
        print(f"q: {q}")  # 打印当前关节位置
        time.sleep(0.02)  # 等待20毫秒
    time.sleep(5)
    set_arm_control_mode(1)
    function_status_pub.publish(True)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass