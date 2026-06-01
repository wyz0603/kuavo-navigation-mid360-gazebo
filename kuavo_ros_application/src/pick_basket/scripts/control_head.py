#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
import rospy
import time
from dynamic_biped.msg import robotHeadMotionData

rospy.init_node('control_head_dem_node')
rospy.loginfo("control head.....")
head_traj_pub = rospy.Publisher("/robot_head_motion_data", robotHeadMotionData, queue_size=10)        

def control_head(yaw, pitch):
        """
        yaw: desired yaw angle in degrees
        pitch: desired pitch angle in degrees
        """    

        # Print the values
        print(f"[Tag] Head pitch: {pitch}")
        print(f"[Tag] Head yaw: {yaw}")

        # Clamp yaw to the range [-30, 30]
        yaw = max(-30.0, min(30.0, yaw))

        # Clamp pitch to the range [-25, 25]
        pitch = max(-25.0, min(25.0, pitch))

        # Create and populate the robotHeadMotionData message
        head_cmd = robotHeadMotionData()
        head_cmd.joint_data = [0.0, 0.0]  # Initialize with zeros
        head_cmd.joint_data[0] = yaw  # yaw in degrees
        head_cmd.joint_data[1] = pitch  # pitch in degrees

        # Publish the command
        head_traj_pub.publish(head_cmd)

if __name__ == "__main__":
    try:
        time.sleep(2.0)
        control_head(0, -15)
        print("control head: pitch 15.....")
    except rospy.ROSInterruptException:
        pass