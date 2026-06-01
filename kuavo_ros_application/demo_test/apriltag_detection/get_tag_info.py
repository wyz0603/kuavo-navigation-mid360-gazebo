#!/usr/bin/env python

import rospy
from apriltag_ros.msg import AprilTagDetectionArray
import math
import numpy as np  # 引入numpy库用于数值计算


class AprilTagProcessor:
    def __init__(self, is_init=False):
        """
        初始化AprilTagProcessor类。
        :param is_init: 是否初始化ROS节点，默认为False。
        """
        if is_init:
            rospy.init_node('tag_detections_listener', anonymous=True)

    def quaternion_to_euler(self, w, x, y, z):
        """
        将四元数转换为欧拉角。
        :param w, x, y, z: 四元数的分量。
        :return: 包含三个欧拉角（roll, pitch, yaw）的元组，单位为度。
        """
        # 计算roll, pitch, yaw
        sinr_cosp = 2 * (w * z + x * y)
        cosr_cosp = 1 - 2 * (y**2 + z**2)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = math.asin(sinp) if abs(sinp) < 1 else math.copysign(math.pi / 2, sinp)

        siny_cosp = 2 * (w * x + y * z)
        cosy_cosp = 1 - 2 * (x**2 + y**2)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        # 转换为角度并返回所有三个角度
        return (math.degrees(roll), math.degrees(pitch), math.degrees(yaw))

    def get_apriltag_data(self):
        """
        从指定的ROS话题中获取AprilTag检测数据。
        :return: 包含每个AprilTag信息的列表。
        """
        if rospy.is_shutdown():
            return None
            
        try:
            msg = rospy.wait_for_message("/robot_tag_info", AprilTagDetectionArray, timeout=5)
        except rospy.ROSInterruptException:
            rospy.loginfo("程序被用户中断")
            return None

        data_list = []
        for detection in msg.detections:
            id = detection.id[0]  # 获取AprilTag的ID
            quaternion = detection.pose.pose.pose.orientation  # 获取姿态的四元数
            pos = detection.pose.pose.pose.position  # 获取位置

            # 获取所有三个欧拉角
            roll_angle, pitch_angle, yaw_angle = self.quaternion_to_euler(
                quaternion.w, quaternion.x, quaternion.y, quaternion.z)

            # 构建AprilTag数据字典
            tag_data = {
                "id": id,
                "off_horizontal": round(pos.x, 3),
                "off_camera": round(pos.y, 3),
                "off_vertical": round(pos.z, 3),
                "roll_angle": roll_angle,
                "pitch_angle": pitch_angle,
                "yaw_angle": yaw_angle
            }
            data_list.append(tag_data)
            rospy.loginfo(f"检测到 AprilTag {id}: 位置(x={pos.x:.3f}, y={pos.y:.3f}, z={pos.z:.3f}), 角度={yaw_angle:.3f}")  # 添加每个标签的信息

        return data_list

    def get_apriltag_by_id(self, tag_id):
        """
        根据ID获取特定的AprilTag数据。
        :param tag_id: 要查找的AprilTag的ID。
        :return: 匹配的AprilTag数据字典。
        """
        all_tags = self.get_apriltag_data()
        if all_tags is None:
            rospy.logerr("未能获取到 AprilTag 数据")
            return None

        for tag in all_tags:
            if tag["id"] == tag_id:
                return tag

        return None

    def get_averaged_apriltag_data(self, tag_id, num_samples=10):
        """
        获取指定ID的AprilTag的平均位置和姿态数据。
        :param tag_id: 要查找的AprilTag的ID。
        :param num_samples: 用于计算平均值的样本数量，默认为10。
        :return: 包含平均位置和姿态的字典。
        """
        data_list = []

        try:
            while len(data_list) < num_samples:
                tag_data = self.get_apriltag_by_id(tag_id)
                if tag_data:
                    data_list.append(tag_data)
                if rospy.is_shutdown():  # 检查ROS是否被关闭
                    return None
        except KeyboardInterrupt:
            rospy.loginfo("程序被用户中断")
            return None

        if not data_list:  # 如果没有收集到数据
            return None

        # 使用numpy计算平均值
        avg_off_horizontal = np.mean([tag["off_horizontal"] for tag in data_list])
        avg_off_camera = np.mean([tag["off_camera"] for tag in data_list])
        avg_off_vertical = np.mean([tag["off_vertical"] for tag in data_list])
        avg_roll_angle = np.mean([tag["roll_angle"] for tag in data_list])
        avg_pitch_angle = np.mean([tag["pitch_angle"] for tag in data_list])
        avg_yaw_angle = np.mean([tag["yaw_angle"] for tag in data_list])

        result = {
            "id": tag_id,
            "avg_off_horizontal": round(avg_off_horizontal, 3),
            "avg_off_camera": round(avg_off_camera, 3),
            "avg_off_vertical": round(avg_off_vertical, 3),
            "avg_roll_angle": round(avg_roll_angle, 3),
            "avg_pitch_angle": round(avg_pitch_angle, 3),
            "avg_yaw_angle": round(avg_yaw_angle, 3)
        }
        rospy.loginfo(f"AprilTag ID: {result['id']}, 位置: x={result['avg_off_horizontal']}, y={result['avg_off_camera']}, z={result['avg_off_vertical']}, 横滚角(Roll): {result['avg_roll_angle']}, 俯仰角(Pitch): {result['avg_pitch_angle']}, 偏航角(Yaw): {result['avg_yaw_angle']}")
        return result


if __name__ == '__main__':
    try:
        # 创建AprilTagProcessor实例并初始化ROS节点
        processor = AprilTagProcessor(is_init=True)
        # 获取指定ID的AprilTag的平均数据
        tag_data = processor.get_averaged_apriltag_data(tag_id=0)
        
        # 将结果保存到文件
        if tag_data:
            with open('apriltag_results.txt', 'w') as f:
                f.write(f"AprilTag检测结果:\n")
                f.write(f"标签ID: {tag_data['id']}\n")
                f.write(f"水平偏移: {tag_data['avg_off_horizontal']} 米\n")
                f.write(f"相机偏移: {tag_data['avg_off_camera']} 米\n")
                f.write(f"垂直偏移: {tag_data['avg_off_vertical']} 米\n")
                f.write(f"横滚角(Roll): {tag_data['avg_roll_angle']} 度\n")
                f.write(f"俯仰角(Pitch): {tag_data['avg_pitch_angle']} 度\n")
                f.write(f"偏航角(Yaw): {tag_data['avg_yaw_angle']} 度\n")
            rospy.loginfo("结果已保存到 apriltag_results.txt")
        else:
            rospy.logerr("未获取到有效数据，无法保存文件")
            
    except KeyboardInterrupt:
        rospy.loginfo("程序被用户中断")
    except rospy.ROSInterruptException:
        pass
