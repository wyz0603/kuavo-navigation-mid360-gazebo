#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Float32, Float32MultiArray
from sensor_msgs.msg import JointState
# from kuavo_ros_interfaces.srv import ocs2ChangeArmCtrlMode
from kuavo_msgs.srv import changeArmCtrlMode
import numpy as np
import time

pub = rospy.Publisher("/kuavo_arm_traj", JointState, queue_size=10)

def call_change_arm_ctrl_mode_service(arm_ctrl_mode):
    result = True
    service_name = "humanoid_change_arm_ctrl_mode"
    try:
        rospy.wait_for_service(service_name, timeout=0.5)
        change_arm_ctrl_mode = rospy.ServiceProxy(
            "humanoid_change_arm_ctrl_mode", changeArmCtrlMode
        )
        change_arm_ctrl_mode(control_mode=arm_ctrl_mode)
        rospy.loginfo("Service call successful")
    except rospy.ServiceException as e:
        rospy.loginfo("Service call failed: %s", e)
        result = False
    except rospy.ROSException:
        rospy.logerr(f"Service {service_name} not available")
        result = False
    finally:
        return result


def publish_joint_states(q_now):
    msg = JointState()
    msg.name = ["arm_joint_" + str(i) for i in range(1, 15)]
    msg.header.stamp = rospy.Time.now()
    msg.position = np.array(q_now)
    # msg.position = 180.0 / np.pi * np.array(q_now)
    pub.publish(msg)

if __name__ == '__main__':
    rospy.init_node('sim_traj', anonymous=True)
    call_change_arm_ctrl_mode_service(2)
    rate = rospy.Rate(10) # 10hz
    q_list = []
    q0 = [0.0]*14
    q1 = [0.0]*14
    # q1[0:7] = [0, 0, 0, 0, 0, 0, 0]
    q1[7:14] = [-90, -0, -0, -90, 90, 0, 0]
    # q1[7:14] = [-0, -0, -0, -0, 0, 0, 0]
    num = 90
    for i in range(num):
        q_tmp = [0.0]*14
        for j in range(14):
            q_tmp[j] = q0[j] + i/float(num)*(q1[j] - q0[j])
        q_list.append(q_tmp)  

    q_list.append(q1)
    for q in q_list:
        publish_joint_states(q)
        print(f"q: {q}")
        time.sleep(0.02)
