#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from apriltag_ros.msg import AprilTagDetectionArray
import math

class RealtimeTagInfoPrinter:
    def __init__(self):
        """初始化实时Tag信息打印器"""
        rospy.init_node('realtime_tag_info_printer', anonymous=True)
        
        # 订阅/robot_tag_info话题
        self.subscriber = rospy.Subscriber('/robot_tag_info', AprilTagDetectionArray, self.tag_callback)
        
        rospy.loginfo("实时Tag信息打印器已启动，正在监听 /robot_tag_info 话题...")
        
    def quaternion_to_yaw(self, w, x, y, z):
        """
        将四元数转换为yaw角（偏航角）
        :param w, x, y, z: 四元数的分量
        :return: yaw角，单位为度
        """
        # 计算yaw角
        siny_cosp = 2 * (w * x + y * z)
        cosy_cosp = 1 - 2 * (x**2 + y**2)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        # 转换为角度
        return math.degrees(yaw)
    
    def tag_callback(self, msg):
        """
        回调函数，处理接收到的AprilTag检测消息
        :param msg: AprilTagDetectionArray消息
        """
        if not msg.detections:
            rospy.loginfo("未检测到任何AprilTag")
            return
            
        rospy.loginfo("=" * 60)
        rospy.loginfo(f"检测到 {len(msg.detections)} 个AprilTag:")
        
        for i, detection in enumerate(msg.detections):
            # 获取Tag ID
            tag_id = detection.id[0] if detection.id else "Unknown"
            
            # 获取位置信息
            pos = detection.pose.pose.pose.position
            x = pos.x
            y = pos.y  
            z = pos.z
            
            # 获取姿态四元数
            quaternion = detection.pose.pose.pose.orientation
            yaw = self.quaternion_to_yaw(quaternion.w, quaternion.x, quaternion.y, quaternion.z)
            
            # 打印信息
            rospy.loginfo(f"  Tag {i+1} (ID: {tag_id}):")
            rospy.loginfo(f"    位置: x={x:.4f}, y={y:.4f}, z={z:.4f}")
            rospy.loginfo(f"    Yaw角: {yaw:.2f}°")
            
        rospy.loginfo("=" * 60)
    
    def run(self):
        """运行节点"""
        try:
            rospy.spin()
        except KeyboardInterrupt:
            rospy.loginfo("程序被用户中断")

def main():
    """主函数"""
    try:
        # 创建并运行实时Tag信息打印器
        printer = RealtimeTagInfoPrinter()
        printer.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS节点被中断")

if __name__ == '__main__':
    main()
