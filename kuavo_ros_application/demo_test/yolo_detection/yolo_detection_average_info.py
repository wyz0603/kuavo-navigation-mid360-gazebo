#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import Point, Quaternion
import os
from datetime import datetime

def normalize_quaternion(quat):
    norm = np.linalg.norm(quat)
    if norm == 0:
        raise ValueError("Cannot normalize a zero-length quaternion")
    return quat / norm
def get_position_and_orientation(sample_count=100):
    positions = []
    orientations = []
    
    for _ in range(sample_count):
        # 等待并获取一次消息
        data = rospy.wait_for_message("/object_yolo_box_tf2_torso_result", Detection2DArray)

        if data.detections:
            detection = data.detections[0]
            positions.append(detection.results[0].pose.pose.position)
            orientations.append(detection.results[0].pose.pose.orientation)
        data = None
    if not positions or not orientations:
        return (None, None)
    avg_position = Point()
    avg_position.x = sum(p.x for p in positions) / len(positions)
    avg_position.y = sum(p.y for p in positions) / len(positions)
    avg_position.z = sum(p.z for p in positions) / len(positions)
    avg_orientation = Quaternion()
    avg_orientation.x = sum(o.x for o in orientations) / len(orientations)
    avg_orientation.y = sum(o.y for o in orientations) / len(orientations)
    avg_orientation.z = sum(o.z for o in orientations) / len(orientations)
    avg_orientation.w = sum(o.w for o in orientations) / len(orientations)

    # 返回 Position 和 Orientation
    return (avg_position, avg_orientation)

def save_detection_results(position=None, file_path="yolo_box_info.txt"):
    """将检测结果保存到文本文件，只保存位置信息"""
    try:
        current_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        
        if position is None:
            content = f"""检测时间: {current_time}
状态: 未检测到有效目标
----------------------------------------
"""
        else:
            content = f"""检测时间: {current_time}
位置信息:
X: {position.x:.4f}
Y: {position.y:.4f}
Z: {position.z:.4f}
----------------------------------------
"""
        # 以写模式打开文件
        with open(file_path, 'w', encoding='utf-8') as f:
            f.write(content)
        rospy.loginfo(f"检测结果已保存到: {file_path}")
    except Exception as e:
        rospy.logerr(f"保存检测结果时出错: {str(e)}")

if __name__ == '__main__':
    rospy.init_node('object_yolo_box_listener', anonymous=True)
    
    # 获取位置和方向信息
    position, orientation = get_position_and_orientation()
    
    if position is not None:
        # 只保存位置信息
        save_detection_results(position)
        rospy.loginfo("检测完成并保存结果")
    else:
        # 保存未检测到目标的结果
        save_detection_results()
        rospy.logwarn("未检测到有效目标")
            