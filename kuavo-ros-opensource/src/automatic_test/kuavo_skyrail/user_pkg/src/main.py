#!/usr/bin/python3
# -*- coding: utf-8 -*-

import rospy
import math
import time
import os
import sys
import threading
import tkinter as tk
import signal

import tf
from tf.transformations import euler_from_quaternion

from geometry_msgs.msg import Pose
from nimservos_controller.msg import motor_cmd
from std_msgs.msg import String
from pid import PIDController
from user_pkg.msg import piddebug, uwbdebug
from std_msgs.msg import Int8

import numpy as np
import scipy.signal as sig
import matplotlib.pyplot as plt

# ------------------------
# 常量定义
# ------------------------
MOVE_SPEED = 50000
POSITION_ERROR_THRESHOLD = 3
CAR_LIMIT_X0 = 0.2
CAR_LIMIT_X1 = 4.1
CAR_LIMIT_Y0 = 1.8
CAR_LIMIT_Y1 = 6.0

def rotate_cw(x, y, angle_degrees):
    """顺时针旋转 (如果不再需要，可删除)"""
    angle_radians = np.deg2rad(angle_degrees)
    rotation_matrix = np.array([
        [np.cos(angle_radians), np.sin(angle_radians)],
        [-np.sin(angle_radians), np.cos(angle_radians)]
    ])
    original_coords = np.array([x, y])
    new_coords = np.dot(rotation_matrix, original_coords)
    return new_coords[0], new_coords[1]

def rotate_ccw(x, y, angle_degrees):
    """逆时针旋转 (如果不再需要，可删除)"""
    angle_radians = np.deg2rad(angle_degrees)
    rotation_matrix = np.array([
        [np.cos(angle_radians), -np.sin(angle_radians)],
        [np.sin(angle_radians),  np.cos(angle_radians)]
    ])
    original_coords = np.array([x, y])
    new_coords = np.dot(rotation_matrix, original_coords)
    return new_coords[0], new_coords[1]

class PositionDataTypedef:
    """用于存储单个设备(车/机器人)的位姿数据"""
    def __init__(self, id):
        self.id = id
        # 上一时刻坐标(如需使用可保留)
        self.last_x = 0
        self.last_y = 0
        self.last_z = 0

        # 当前时刻坐标
        self.cur_x = 0
        self.cur_y = 0
        self.cur_z = 0

        # 欧拉角
        self.yaw   = 0
        self.pitch = 0
        self.roll  = 0
        self.poslists = [[],[]]
        self.poslist_lengthX = 10  #100
        self.poslist_lengthY = 10 #120


class UserNode:

    def __init__(self):
        # 初始化 ROS 节点
        rospy.init_node(name="user", anonymous=True)

        # 并发锁，防止多线程读写冲突
        self.updatelock = threading.Lock()

        # 初始化 car 和 robot (ID可自行定义)
        self.car   = PositionDataTypedef(id=2)
        self.car.poslist_lengthX = 5
        self.car.poslist_lengthY = 1
        self.robot = PositionDataTypedef(id=1)
        self.robot.poslist_lengthX = 10
        self.robot.poslist_lengthY = 1

        # 发布者
        self.uwbdebug_pub   = rospy.Publisher("/uwb_debug",   uwbdebug,         queue_size=100)
        self.piddebug_pub   = rospy.Publisher("/pid_debug",   piddebug,         queue_size=10)
        self.nimservos_pub  = rospy.Publisher("/nimmotor_control", motor_cmd,   queue_size=10)
        self.auto_tracking_stats_pub = rospy.Publisher("/auto_tracking_stats", Int8, queue_size=10)

        # 位移指令，用于电机控制
        self.move_x_distance = 0
        self.move_y_distance = 0

        # PID 控制器
        self.pidcontroller_x = PIDController()
        self.pidcontroller_y = PIDController()
        self.pidcontroller_offsetx = PIDController()
        self.pidcontroller_offsety = PIDController()

        self.car_pid_x = 0
        self.car_pid_y = 0
        self.robot_pid_x = 0
        self.robot_pid_y = 0

        # 直接订阅 Pose 类型，而非 AnyMsg
        rospy.Subscriber("/car_pose",   Pose, self.car_pose_cb)
        rospy.Subscriber("/robot_pose", Pose, self.robot_pose_cb)

        # 界面相关
        self.control_mode = "键盘控制模式"
        self.root = tk.Tk()
        self.master = self.root
        self.master.title("Key Listener")

        self.control_mode_label = tk.Label(self.root, text=self.control_mode, font=("Helvetica", 16))
        self.control_mode_label.pack(pady=20)

        self.control_tip_label = tk.Label(self.root, text="WSAD 控制天轨前后左右移动", font=("Helvetica", 16))
        self.control_tip_label.pack(pady=20)

        self.globel_label = tk.Label(self.root, text="模式切换: o -> 键盘控制;  p -> UWB控制", font=("Helvetica", 10))
        self.globel_label.pack(pady=20)

        self.key_status_label = tk.Label(self.root, text="No key pressed", font=("Helvetica", 14))
        self.key_status_label.pack(pady=20)

        self.master.bind("<KeyPress>", self.on_key_press)

        self.last_key_press_time = {}
        self.key_timeout = 0.5
        self.keys_pressed = {}

        # 启动一个线程来 spin，保证订阅回调可以执行
        self.spin_thread = threading.Thread(target=self.spin_loop)
        self.spin_thread.start()

        # 线程：周期发布电机指令
        self.nimservos_cmdpub_thread = threading.Thread(target=self.nimservor_cmdpub)
        self.nimservos_cmdpub_thread.start()

    def spin_loop(self):
        """在单独线程中调用 rospy.spin()，使回调有机会执行"""
        rospy.spin()
        rospy.loginfo("ROS spin thread exited.")

    # ======================
    # 订阅回调函数
    # ======================
    def car_pose_cb(self, pose_msg: Pose):
        """处理 /car_pose 的回调"""
        with self.updatelock:
            tmp_x = pose_msg.position.x
            tmp_y = pose_msg.position.y
            tmp_z = pose_msg.position.z
            # self.car.cur_x = pose_msg.position.x
            # self.car.cur_y = pose_msg.position.y
            self.car.cur_z = pose_msg.position.z
            self.car.poslists[0].insert(0, tmp_x)
            self.car.poslists[1].insert(0, tmp_y)
            # 分别计算X和Y轴数据的数量
            data_count_x = len(self.car.poslists[0])
            data_count_y = len(self.car.poslists[1])
            if data_count_x > self.car.poslist_lengthX:
                    self.car.poslists[0].pop()  # 移除最旧的X坐标
                    self.car.cur_x = sum(self.car.poslists[0]) / min(data_count_x, self.car.poslist_lengthX)

                    # 对Y轴进行滑动平均滤波
            if data_count_y > self.car.poslist_lengthY:
                    self.car.poslists[1].pop()  # 移除最旧的Y坐标
                    self.car.cur_y = sum(self.car.poslists[1]) / min(data_count_y, self.car.poslist_lengthY)


            # 四元数 -> 欧拉角
            ori = pose_msg.orientation
            roll, pitch, yaw = euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])
            self.car.roll  = roll
            self.car.pitch = pitch
            self.car.yaw   = yaw

            # 这里使用您脚本中自定义的坐标转换，如：(-car.cur_y, car.cur_x)
            self.car_pid_x =  self.car.cur_x - 500
            self.car_pid_y =  self.car.cur_y

            self.uwbdebug_msg_send()

    def robot_pose_cb(self, pose_msg: Pose):
        """处理 /robot_pose 的回调"""
        with self.updatelock:
            tmp_x = pose_msg.position.x
            tmp_y = pose_msg.position.y
            tmp_z = pose_msg.position.z
            
            # self.robot.cur_x = pose_msg.position.x
            # self.robot.cur_y = pose_msg.position.y
            self.robot.cur_z = pose_msg.position.z
            self.robot.poslists[0].insert(0, tmp_x)
            self.robot.poslists[1].insert(0, tmp_y)

            data_count_x = len(self.robot.poslists[0])
            data_count_y = len(self.robot.poslists[1])
            if data_count_x > self.robot.poslist_lengthX:
                    self.robot.poslists[0].pop()  # 移除最旧的X坐标
                    self.robot.cur_x = sum(self.robot.poslists[0]) / min(data_count_x, self.robot.poslist_lengthX)

                    # 对Y轴进行滑动平均滤波
            if data_count_y > self.robot.poslist_lengthY:
                    self.robot.poslists[1].pop()  # 移除最旧的Y坐标
                    self.robot.cur_y = sum(self.robot.poslists[1]) / min(data_count_y, self.robot.poslist_lengthY)
            ori = pose_msg.orientation
            roll, pitch, yaw = euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])
            self.robot.roll  = roll
            self.robot.pitch = pitch
            self.robot.yaw   = yaw

            # 同样做坐标转换
            self.robot_pid_x =  self.robot.cur_x
            self.robot_pid_y =  self.robot.cur_y

            self.uwbdebug_msg_send()

    # ======================
    # 发布电机指令
    # ======================
    def nimservos_cmd_send(self):
        msg = motor_cmd()
        with self.updatelock:
            # 判断 car 是否无效（无数据或跑飞值）
            car_no_data = (self.car.cur_x == 0 and self.car.cur_y == 0 and self.car.cur_z == 0)
            car_out_of_range = (abs(self.car.cur_x) > 8000 or 
                                abs(self.car.cur_y) > 8000 or 
                                abs(self.car.cur_z) > 8000)
            invalid_car = (car_no_data or car_out_of_range)
            
            # 判断 robot 是否无效（无数据或跑飞值）
            robot_no_data = (self.robot.cur_x == 0 and self.robot.cur_y == 0 and self.robot.cur_z == 0)
            robot_out_of_range = (abs(self.robot.cur_x) > 8000 or 
                                abs(self.robot.cur_y) > 8000 or 
                                abs(self.robot.cur_z) > 8000)
            invalid_robot = (robot_no_data or robot_out_of_range)

            # 如果 car 或者 robot 中任何一个数据无效，则认为定位异常，停止电机，并记录日志
            if invalid_car or invalid_robot:
                msg.linear_x = 0
                msg.linear_y = 0
                rospy.logwarn("定位模块 异常！！！！！！！！！！！！！！！！！！！！！！！！！")
                self.auto_tracking_stats_pub.publish(0)
            else:
                # 如果数据均正常，则按照原先的业务逻辑发布电机运动指令
                msg.linear_x = self.move_x_distance
                msg.linear_y = self.move_y_distance
                self.auto_tracking_stats_pub.publish(1)
        self.nimservos_pub.publish(msg)


    def piddebug_msg_send(self, objectname, pidcontroller):
        msg = piddebug()
        msg.objectname = objectname
        msg.target = pidcontroller.target
        msg.rev    = pidcontroller.rev
        msg.kp     = pidcontroller.kp
        msg.ki     = pidcontroller.ki
        msg.kd     = pidcontroller.kd
        msg.p_out  = pidcontroller.p_out
        msg.i_out  = pidcontroller.i_out
        msg.d_out  = pidcontroller.d_out
        msg.pid_out= pidcontroller.pid_out
        self.piddebug_pub.publish(msg)

    def uwbdebug_msg_send(self):
        """发布 car/robot 的调试信息，类似原先的 /uwb_debug"""
        msg = uwbdebug()
        msg.car_cur_x = self.car.cur_x
        msg.car_cur_y = self.car.cur_y + 450
        msg.car_cur_z = self.car.cur_z
        msg.car_yaw   = self.car.yaw
        msg.car_pitch = self.car.pitch
        msg.car_roll  = self.car.roll

        msg.robot_cur_x = self.robot.cur_x
        msg.robot_cur_y = self.robot.cur_y
        msg.robot_cur_z = self.robot.cur_z
        msg.robot_yaw   = self.robot.yaw
        msg.robot_pitch = self.robot.pitch
        msg.robot_roll  = self.robot.roll

        self.uwbdebug_pub.publish(msg)

    # ======================
    # GUI + 键盘事件
    # ======================
    def on_key_press(self, event):
        key = event.keysym
        self.key_status_label.config(text=f"Key '{key}' pressed")
        self.keys_pressed[key] = True
        self.last_key_press_time[key] = time.time()

        # 前后左右
        if key in ["w", "W"]:
            self.move_y_distance = MOVE_SPEED
        elif key in ["s", "S"]:
            self.move_y_distance = -MOVE_SPEED
        if key in ["a", "A"]:
            self.move_x_distance = MOVE_SPEED
        elif key in ["d", "D"]:
            self.move_x_distance = -MOVE_SPEED

        # 空格停止
        if key == "space":
            self.move_x_distance = 0
            self.move_y_distance = 0

        # 模式切换
        if key in ["o", "O"]:
            self.control_mode = "键盘控制模式"
            self.move_x_distance = 0
            self.move_y_distance = 0
            self.control_mode_label.config(text=self.control_mode)
            self.control_tip_label.config(text="WSAD 控制天轨前后左右移动")
        elif key in ["p", "P"]:
            self.control_mode = "UWB追踪模式"
            self.control_mode_label.config(text=self.control_mode)
            self.control_tip_label.config(
                text=f"robot id:{self.robot.id} car id:{self.car.id}"
            )

    # ======================
    # 线程：循环发布电机指令
    # ======================
    def nimservor_cmdpub(self):
        rate = rospy.Rate(200)  # 200Hz
        while not rospy.is_shutdown():
            if self.control_mode == "键盘控制模式":
                # 按键松开超时处理
                current_time = time.time()
                for key, is_pressed in list(self.keys_pressed.items()):
                    if is_pressed:
                        last_press = self.last_key_press_time.get(key, 0)
                        if current_time - last_press > self.key_timeout:
                            if key in ["w", "s", "W", "S"]:
                                self.move_y_distance = 0
                            elif key in ["a", "d", "A", "D"]:
                                self.move_x_distance = 0
                            self.key_status_label.config(text=f"Key '{key}' released")
                            self.keys_pressed[key] = False

                self.nimservos_cmd_send()

            elif self.control_mode == "UWB追踪模式":
                # 计算位置误差
                with self.updatelock:
                    error_x = abs(self.robot_pid_x - self.car_pid_x)
                    error_y = abs(self.robot_pid_y - self.car_pid_y)

                distancey = error_y
                distancex = error_x

                if distancey >= 2000 or distancex >= 2000:
                    # 偏差过大，停止
                    self.move_x_distance = 0
                    self.move_y_distance = 0
                    self.pidcontroller_x.pid_clear()
                    self.pidcontroller_y.pid_clear()
                    rospy.logwarn("距离误差过大,停止UWB控制模式")
                    self.nimservos_cmd_send()
                else:
                    # Y 方向
                    if distancey <= 80:
                        self.move_y_distance = 0
                        self.pidcontroller_y.pid_clear()
                    else:
                        # 这里配置 PID 参数
                        self.pidcontroller_y.pid_config(kp=350, ki=0, kd=5000)
                        with self.updatelock:
                            out_y = self.pidcontroller_y.pid_calcul(
                                self.robot_pid_y, self.car_pid_y
                            )
                        # 如果需要 Y 方向运动，取消注释
                        self.move_y_distance = int(out_y)

                    # X 方向
                    if distancex <= 80:
                        self.move_x_distance = 0
                        self.pidcontroller_x.pid_clear()
                    else:
                        self.pidcontroller_x.pid_config(kp=325, ki=0, kd=500)
                        with self.updatelock:
                            out_x = self.pidcontroller_x.pid_calcul(
                                self.robot_pid_x, self.car_pid_x
                            )
                        self.move_x_distance = int(-out_x)

                    self.piddebug_msg_send("天轨x轴", self.pidcontroller_x)
                    self.nimservos_cmd_send()

            rate.sleep()

def signal_handler(sig, frame):
    rospy.loginfo("Ctrl+C detected, shutting down...")
    rospy.signal_shutdown("User requested shutdown")

if __name__ == '__main__':
    try:
        user = UserNode()
        # 捕获 Ctrl+C
        signal.signal(signal.SIGINT, signal_handler)
        # 在主线程跑 Tkinter
        user.root.mainloop()
    except rospy.ROSException as e:
        rospy.loginfo(e)
