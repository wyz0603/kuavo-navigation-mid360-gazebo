#! /usr/bin/env python

import rospy
from apriltag_ros.msg import AprilTagDetectionArray
from scipy.spatial.transform import Rotation
import numpy as np

# config:
orientation_tolerance = 0.005
degree_tolerance = 2.5

def normalize_rpy(rpy):
    """
    规范化欧拉角（RPY），使其在 [-π, π] 范围内
    """
    roll, pitch, yaw = rpy
    roll = (roll + np.pi) % (2 * np.pi) - np.pi
    pitch = (pitch + np.pi) % (2 * np.pi) - np.pi
    yaw = (yaw + np.pi) % (2 * np.pi) - np.pi
    return np.array([roll, pitch, yaw])

def rpy_degree(rpy:list)->list:    
    return [r * 180.0 / np.pi for r in rpy] 

class ARCaliToolNode(object):
    def __init__(self):
        rospy.init_node('apriltag_cali_node')
        self.subscription = rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self.tag_callback)

    def tag_callback(self, msg):
        for detection in msg.detections:
            # Make sure the detection's pose has a valid frame_id
            if detection.pose.header.frame_id == '':
                detection.pose.header.frame_id = 'camera_color_optical_frame'

            x = detection.pose.pose.pose.orientation.x
            y = detection.pose.pose.pose.orientation.y
            z = detection.pose.pose.pose.orientation.z
            w = detection.pose.pose.pose.orientation.w
            quat = [x, y, z, w]
            # rpy = Rotation.from_quat(quat).as_euler('yxz', degrees=False)
            
            # print(f'tag position(x, y, z): {x}, {y}, {z}')
            # print(f'tag rpy:{rpy}')

            # if abs(y) < orientation_tolerance and abs(z) < orientation_tolerance:
            #     quat = [x, y, z, w]
            #     print(f'Found orientation:{detection.pose.pose.pose.orientation}')
            #     print(f'Found rpy:{rpy}')
            #     _ = input("-------- 找到 rpy ,按下 Ctrl + C 结束 -------------")

            rpy = Rotation.from_quat(quat).as_euler('xyz', degrees=False)
            rpy = normalize_rpy(rpy)
            r, p, y = rpy_degree(rpy)
            if abs(r) < degree_tolerance and abs(y) < degree_tolerance:
                quat = [x, y, z, w]
                print(f'Found orientation:{detection.pose.pose.pose.orientation}')
                print(f'Found rpy:{rpy}')
                _ = input("-------- 找到 rpy ,按下 Ctrl + C 结束 -------------")

def main():
    try:
        ar_cali_node = ARCaliToolNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()

