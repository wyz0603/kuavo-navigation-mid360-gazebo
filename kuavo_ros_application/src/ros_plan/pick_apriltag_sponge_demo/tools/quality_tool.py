#! /usr/bin/env python

import rospy
from apriltag_ros.msg import AprilTagDetection, AprilTagDetectionArray
import math

# config:
y_2_to_0 = 0.083  # ID 2 到 ID 0 的 y 距离 7.9 cm
x_2_to_1 = 0.079  # ID 2 到 ID 1 的 x 距离 8.3 cm

def print_ascii_text_good():
    print("\033[92m" '''
   _____                 _ 
  / ____|               | |
 | |  __  ___   ___   __| |
 | | |_ |/ _ \ / _ \ / _` |
 | |__| | (_) | (_) | (_| |
  \_____|\___/ \___/ \__,_|                       
    '''"\033[0m")

def print_ascii_text_bad():
    print("\033[91m" '''
  ____            _ 
 |  _ \          | |
 | |_) | __ _  __| |
 |  _ < / _` |/ _` |
 | |_) | (_| | (_| |
 |____/ \__,_|\__,_|             
    '''"\033[0m")

class ARCaliToolNode(object):
    def __init__(self):
        rospy.init_node('apriltag_cali_node')
        self.subscription = rospy.Subscriber('/robot_tag_info', AprilTagDetectionArray, self.tag_callback)

    def tag_callback(self, msg):
        
        tag0_xyz = [0,0,0]
        tag1_xyz = [0,0,0]
        tag2_xyz = [0,0,0]
        find_tag0 = False
        find_tag1 = False
        find_tag2 = False
        for detection in msg.detections:
            if detection.id[0] == 0:
                tag0_xyz = [detection.pose.pose.pose.position.x, detection.pose.pose.pose.position.y, detection.pose.pose.pose.position.z]
                find_tag0 = True
            elif detection.id[0] == 1:
                tag1_xyz = [detection.pose.pose.pose.position.x, detection.pose.pose.pose.position.y, detection.pose.pose.pose.position.z]
                find_tag1 = True
            elif detection.id[0] == 2:
                tag2_xyz = [detection.pose.pose.pose.position.x, detection.pose.pose.pose.position.y, detection.pose.pose.pose.position.z]
                find_tag2 = True

        if not find_tag2:
            print("未发现 ID 2 标签, 请移动测试纸到相机可视范围内")
            return
        
        if not find_tag0 or not find_tag1:
            print("未发现 ID 0 或 ID 1 标签, 请移动测试纸到相机可视范围内")
            return

        mse_tag20  = 0.0
        mse_tag21  = 0.0
        # Tag2 与 Tag0 的 y,z 应该接近一样
        # Tag2 与 Tag0 的 x 应该相差接近于测量的距离
        diff_tag2_0 = [0.0, 0.0, 0.0]
        diff_tag2_0[0] = abs(tag2_xyz[0] - tag0_xyz[0])
        diff_tag2_0[1] = abs(tag2_xyz[1] - tag0_xyz[1] - y_2_to_0)
        diff_tag2_0[2] = abs(tag2_xyz[2] - tag0_xyz[2])

        # Tag2 与 Tag1 的 y,x 应该接近一样
        # Tag2 与 Tag1 的 y 应该相差接近于测量的距离
        diff_tag2_1 = [0.0, 0.0, 0.0]
        diff_tag2_1[0] = abs(tag2_xyz[0] - tag1_xyz[0] - x_2_to_1)
        diff_tag2_1[1] = abs(tag2_xyz[1] - tag1_xyz[1])
        diff_tag2_1[2] = abs(tag2_xyz[2] - tag1_xyz[2])

        average_diff = [(a + b) / 2.0 for a, b in zip(diff_tag2_0, diff_tag2_1)]
        print(f"tag 2 to tag 0 误差: {diff_tag2_0}")
        print(f"tag 2 to tag 1 误差: {diff_tag2_1}")
        print(f"误差: {average_diff}")

        ok = True    

        if average_diff[0] > 0.015 or average_diff[1] > 0.015 or average_diff[2] > 0.020: # 1.5cm
            ok = False
        if ok:
            print_ascii_text_good()
            _ = input("-------- 按下 Ctrl + C 结束 -------------")   
        else:
            print_ascii_text_bad()
            _ = input("误差不再可接受范围内， 请进行相机或机器人调整，按下 Ctrl + C 结束 -------------") 
def main():
    try:
        ar_cali_node = ARCaliToolNode()
        print("请将测试纸放置在水平桌面上，并按箭头指示，相对于机器人摆正")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()

