#!/opt/miniconda3/envs/joint_cali/bin/python
# -*- coding: utf-8 -*-


import rospy
from kuavo_msgs.msg import robotHeadMotionData
from arm_kinematics import HeadKinematics, get_package_path
import numpy as np
import os

class TargetTracker:
    def __init__(self, urdf_path, pitch_bias=np.deg2rad(-25)):
        self.pitch_bias = pitch_bias
        self.pub = rospy.Publisher("/robot_head_motion_data", robotHeadMotionData, queue_size=10)
        head_kinematics = HeadKinematics(urdf_path)
        pos_i, rot_i = head_kinematics.FK(np.zeros(2))
        self.camera_pos = pos_i
        print(f"camera_pos: {self.camera_pos}")
        
        # 添加跟踪状态标志
        self.tracking_enabled = False
        # 添加跟踪目标位置
        self.target_position = None
        # 添加跟踪定时器
        self.tracking_timer = None

    def set_tracking(self, enable=True):  # 重命名方法，避免与属性冲突
        """启用或禁用头部追踪功能"""
        self.tracking_enabled = enable
    
    def track_target(self, target_pos):
        if not self.tracking_enabled:
            # print("Head tracking is disabled!!!")
            return
        assert len(target_pos) == 3
        q_head = self.get_head_joint_data(target_pos)
        # 转换为角度
        q_head = np.rad2deg(q_head)
        q_head[1] = q_head[1] + np.rad2deg(self.pitch_bias)
        # print(f"q_head: {q_head}")
        msg = self.get_head_msg(q_head)
        self.pub.publish(msg)
    
    def back_to_zero(self):
        q_head = np.zeros(2)
        msg = self.get_head_msg(q_head)
        for i in range(10):
            self.pub.publish(msg)
            rospy.sleep(0.02)

    def get_head_joint_data(self, target_pos):
        assert len(target_pos) == 3
        pos_delta = target_pos - self.camera_pos
        # print(f"pos_delta: {pos_delta}")
        # tan(yaw) = y_delta / x_delta
        yaw = np.arctan2(pos_delta[1], pos_delta[0])
        # tan(pitch) = z_delta / x_delta
        pitch = -np.arctan2(pos_delta[2], pos_delta[0])
        q_head = np.array([yaw, pitch])
        return q_head
    
    def get_head_msg(self, q_head):
        # 限制
        q_head[0] = np.clip(q_head[0], -80, 80)
        q_head[1] = np.clip(q_head[1], -25, 25)
        msg = robotHeadMotionData()
        # 直接赋值而不是使用resize
        msg.joint_data = [q_head[0], q_head[1]]
        # print(f"msg.joint_data: {msg.joint_data}")
        return msg
    
    # def enable_tracking(self, enable):
    #     self.enable_tracking = enable


if __name__ == '__main__':
    rospy.init_node('target_tracker', anonymous=True)
    asset_path = get_package_path("kuavo_assets")
    print(f"asset_path: {asset_path}")
    robot_version = os.environ.get('ROBOT_VERSION', '40')
    urdf_path = os.path.join(asset_path, f"models/biped_s{robot_version}/urdf/biped_s{robot_version}.urdf")
    print(f"urdf_path: {urdf_path}")
    target_tracker = TargetTracker(urdf_path)
    # wait for 1 seconds
    rospy.sleep(1)
    target_tracker.back_to_zero()
    # target_tracker.set_tracking(True)
    for i in range(3):
        target_tracker.track_target(np.array([2.0, -1.0, 1]))
        rospy.sleep(0.1)