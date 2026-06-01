#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import json
import math
import os
import rospkg
import rospy
import cv2
from std_srvs.srv import Empty, Trigger, SetBool
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from ultralytics import YOLO
from roban2_move_boxes.msg import RobotActionState
from roban2_move_boxes.srv import ExecuteArmAction
from kuavo_msgs.msg import robotHeadMotionData
import tf
from nav_msgs.msg import Odometry
import rospy
from std_msgs.msg import String
import sys
import tty
import termios
import select



class MoveBoxes:
    def __init__(self):
        rospy.init_node('move_boxes', anonymous=True)

        # 创建 joy 订阅者，订阅 joy 消息
        self.joy_pub = rospy.Publisher('/joy', Joy, queue_size=10)
        self.joy_msg = Joy()
        # 填充初值
        self.joy_msg.axes = [0.0] * 8
        self.joy_msg.buttons = [0] * 11

        self.is_grad_boxes = False
        self.is_enable_move = False
        self.is_grad_finish = False

        # 获取当前 ros 包的中的模型路径
        self.package_path = rospkg.RosPack().get_path('roban2_move_boxes')
        model_path = os.path.join(self.package_path, 'model/best.pt')
        self.model = YOLO(model_path)

        # 创建一个 CvBridge 对象用于 ROS 图像与 OpenCV 图像之间的转换
        self.bridge = CvBridge()

        # 创建 Twist 消息对象
        self.twist = Twist()
        self.pose_twist = Twist()

        # 图像订阅者，订阅原始图像话题
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.image_callback)

        # 图像发布者，发布处理后的图像
        self.image_pub = rospy.Publisher("/camera/color/image_processed", Image, queue_size=10)

        # 头控制发布
        self.head_pub = rospy.Publisher('/robot_head_motion_data', robotHeadMotionData, queue_size=1)

        self.pose_pub = rospy.Publisher('/cmd_pose', Twist, queue_size=10)

        # 创建 cmd_vel 话题发布者
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # 抓取动作服务
        self.grab_boxes = rospy.ServiceProxy('/execute_arm_action', ExecuteArmAction)

        # 键盘 的发布和订阅
        self.key_pub = rospy.Publisher('/key_pressed', String, queue_size=10)
        self.key_sub = rospy.Subscriber('/key_pressed', String, self.keyboard_callback)

        # self.robot_action_state_pub = rospy.Publisher('/robot_action_state', RobotActionState, queue_size=1)
        # 订阅'/robot_action_state'
        self.robot_action_state_sub = rospy.Subscriber('/robot_action_state', RobotActionState,self.robot_action_state_callback)


        self.offset_x = 0
        self.offset_y = 0
        self.area_ratio = 0
        self.action_state = 0

    def robot_action_state_callback(self, msg):
        rospy.loginfo(f"robot_action_state_callback :{msg.state}")
        self.action_state = msg.state
        pass

    def image_callback(self, data):

        if True:
            rate = rospy.Rate(30)
            try:
                # 将 ROS Image 转换为 OpenCV 格式
                cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
                frame = cv_image

                # 运行模型
                results = self.model.predict(frame, conf=0.80, classes=[0], verbose=False)

                # 创建一个空列表，用于存储多个箱子的位置和大小
                offset_x_list = []
                offset_y_list = []
                area_ratio_list = []

                # 获取所有检测框
                boxes = results[0].boxes.xyxy.cpu().numpy()  # 坐标

                # 绘制边界框和中心
                for box in boxes:
                    cv2.rectangle(frame, (int(box[0]), int(box[1])), (int(box[2]), int(box[3])), (0, 255, 0), 2)
                    cv2.circle(frame, (int(box[0] + (box[2] - box[0]) / 2), int(box[1] + (box[3] - box[1]) / 2)), 5,
                               (0, 0, 255), -1)

                    # 计算矩形框中心和图像中心之间的偏移量，由于rgb摄像头不是位于摄像头正中的位置，所以需要加入70的偏置
                    offset_x, offset_y = box[0] + (box[2] - box[0]) / 2 - (frame.shape[1] / 2 + 70), box[1] + (
                            box[3] - box[1]) / 2 - frame.shape[0] / 2

                    # 计算矩形框的面积比例
                    area_ratio = ((box[2] - box[0]) * (box[3] - box[1])) / (frame.shape[1] * frame.shape[0])
                    # rospy.loginfo("面积比: {:.2f} %".format(area_ratio * 100))

                    # 保存到列表中
                    offset_x_list.append(offset_x)
                    offset_y_list.append(offset_y)
                    area_ratio_list.append(area_ratio)

                    cv2.putText(frame, f'Offset X: {offset_x:0.0f}', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                                (0, 0, 255),
                                2)
                    cv2.putText(frame, f'Offset Y: {offset_y:0.0f}', (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                                (0, 0, 255),
                                2)
                    cv2.putText(frame, f'Area Ratio: {area_ratio * 100:0.2f} %', (10, 90), cv2.FONT_HERSHEY_SIMPLEX,
                                0.7,
                                (0, 0, 255), 2)

                center_x = frame.shape[1] // 2 + 70
                center_y = frame.shape[0] // 2

                # 绘制图像中心坐标系
                cv2.line(frame, (center_x, 0), (center_x, frame.shape[0]), (0, 0, 255), 2)
                cv2.line(frame, (0, center_y), (frame.shape[1], center_y), (0, 0, 255), 2)

                # # 显示原始图像处理结果
                # cv2.imshow('result', frame)
                # cv2.waitKey(1)  # 等待 1ms 来刷新窗口

                # 控制机器人运动，检测到多个箱子先搬左边的那个
                if offset_x_list == []:
                    offset_x_list.append(0)
                    offset_y_list.append(0)
                    area_ratio_list.append(0)
                else:
                    # offset_x_list按 x 的升序排序， offset_y_list 和 area_ratio_list按照 offset_x_list变换的顺序进行排序
                    offset_x_list, offset_y_list, area_ratio_list = zip(
                        *sorted(zip(offset_x_list, offset_y_list, area_ratio_list)))
                    pass

                # 获取最左边箱子的位置和大小
                self.offset_x = offset_x_list[0]
                self.offset_y = offset_y_list[0]
                self.area_ratio = area_ratio_list[0]

            except CvBridgeError as e:
                rospy.logerr(e)
                return

            try:
                # 将处理后的 OpenCV 图像转回 ROS Image 消息并发布
                ros_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                self.image_pub.publish(ros_msg)
                pass
            except CvBridgeError as e:
                rospy.logerr(e)

            rate.sleep()

    def keyboard_callback(self, data):
        self.key = data.data
        # 如果等于keyboard_key为空格，则执行抓取动作
        if self.key == ' ':
            if not self.is_enable_move:
                # 步态切换
                self.is_enable_move = True
                self.joy_msg.buttons[0] = 0
                self.joy_msg.buttons[3] = 1
                self.joy_pub.publish(self.joy_msg)

                rospy.loginfo("Start moving")
            else:
                self.is_enable_move = False

                # 步态切换
                # 获取当前self.twist.linear.x，逐渐减小，最终变为0 ，
                def adjust_value(value):
                    if value > 0:
                        value -= 0.1
                        if value < 0:
                            value = 0.0
                    else:
                        value += 0.1
                        if value > 0:
                            value = 0.0
                    return value

                while self.twist.linear.x != 0 or self.twist.angular.z != 0:
                    self.twist.linear.x = adjust_value(self.twist.linear.x)
                    self.twist.angular.z = adjust_value(self.twist.angular.z)

                    self.cmd_vel_pub.publish(self.twist)
                    rospy.sleep(0.5)

                self.joy_msg.buttons[0] = 1
                self.joy_msg.buttons[3] = 0
                self.joy_pub.publish(self.joy_msg)

                rospy.loginfo("Stop moving")

    def robot_moves_back(self):
        """
        使机器人转动并后退。

        设置机器人的线速度和角速度，
        使机器人在一定时间内转动。之后，改变机器人的线速度，使其前进，
        同时保持角速度为零，以使机器人在转动后直行一段距离。最后，停止机器人。
        """
        rospy.loginfo("robot moves back")

        # 初始化循环频率为10Hz
        rate = rospy.Rate(10)  # 10Hz

        # 设置机器人后退并转动
        self.twist.linear.x = 0.0
        self.twist.angular.z = math.radians(14)
        start_time = rospy.Time.now().to_sec()

        # 持续发布机器人速度命令，使机器人后退并转动13秒
        while not rospy.is_shutdown() and (rospy.Time.now().to_sec() - start_time) < 13:
            self.cmd_vel_pub.publish(self.twist)
            rate.sleep()

        # 设置机器人前进，同时停止转动
        self.twist.linear.x = 0.2
        self.twist.angular.z = math.radians(0.0)

        start_time = rospy.Time.now().to_sec()

        # 持续发布机器人速度命令，使机器人前进5秒
        while not rospy.is_shutdown() and (rospy.Time.now().to_sec() - start_time) < 5:
            self.cmd_vel_pub.publish(self.twist)
            rate.sleep()

        # 停止机器人
        self.twist.linear.x = 0.0
        self.twist.angular.z = math.radians(0.0)
        self.cmd_vel_pub.publish(self.twist)
        pass

    def grad_boxes(self):
        """
        控制机械臂执行抓取箱子的动作，并处理抓取结果。

        本函数首先将抓取请求标志设置为True，然后调用名为'roban2_arm_action'的服务。
        如果服务调用成功，将完成抓取动作，并更新相关状态。
        """
        # 发送一个抓取请求
        self.is_grad_boxes = True
        service_name = '/execute_arm_action'

        # 等待服务变为可用状态，超时设置为5秒
        arm_action_name = "roban2_move_boxes"
        rospy.wait_for_service(service_name, timeout=5)

        try:
            # 调用抓取箱子的服务
            response = self.grab_boxes(arm_action_name)

            while self.action_state != 2:
                rospy.sleep(0.5)

            if response.success:
                rospy.loginfo(f"动作执行成功: {response.message}")
                return True
            else:
                rospy.logerr(f"动作执行失败: {response.message}")
                return False

        except rospy.ServiceException as e:
            # 服务调用失败时，记录错误信息
            rospy.logerr(f"${arm_action_name} :Service call failed: {e}")
            return False

        # 抓取完成后，更新抓取完成标志为True
        self.is_grad_finish = True

        # 记录抓取结果
        rospy.loginfo(f"结果: {response.message}")

        # 重置抓取请求标志为False
        self.is_grad_boxes = False

    def robot_moves_go(self):

        if self.is_enable_move and not self.is_grad_boxes and not self.is_grad_finish:

            if self.offset_x > 100:
                rospy.loginfo("Turn right")
                self.twist.linear.x = 0.0
                self.twist.angular.z = -0.1
                self.cmd_vel_pub.publish(self.twist)
            elif self.offset_x < -100:
                rospy.loginfo("Turn left")
                self.twist.linear.x = 0.0
                self.twist.angular.z = 0.1
                self.cmd_vel_pub.publish(self.twist)
            else:

                if self.offset_y < 50:
                    rospy.loginfo(f"Fast Move forward")
                    self.twist.linear.x = 0.2
                    self.twist.angular.z = 0.0
                    self.cmd_vel_pub.publish(self.twist)

                elif self.offset_y < 70:
                    rospy.loginfo(f"Move forward")
                    self.twist.linear.x = 0.1
                    self.twist.angular.z = 0.0
                    self.cmd_vel_pub.publish(self.twist)

                else:
                    # 停止机器人运动
                    if self.area_ratio > 0.15 and self.area_ratio < 0.35:
                        # 开始进行抓取动作
                        def adjust_value(value):
                            if value > 0:
                                value -= 0.1
                                if value < 0:
                                    value = 0.0
                            else:
                                value += 0.1
                                if value > 0:
                                    value = 0.0
                            return value

                        while self.twist.linear.x != 0 or self.twist.angular.z != 0:
                            self.twist.linear.x = adjust_value(self.twist.linear.x)
                            self.twist.angular.z = adjust_value(self.twist.angular.z)

                            self.cmd_vel_pub.publish(self.twist)
                            rospy.sleep(0.5)

                        self.joy_msg.buttons[0] = 0
                        self.joy_msg.buttons[3] = 1
                        self.joy_pub.publish(self.joy_msg)

                        rospy.sleep(1)

                        self.joy_msg.buttons[0] = 1
                        self.joy_msg.buttons[3] = 0
                        self.joy_pub.publish(self.joy_msg)

                        rospy.sleep(1)
                        rospy.loginfo("squat")
                        self.pose_twist.linear.z = -0.15
                        self.pose_pub.publish(self.pose_twist)
                        rospy.sleep(0.5)

                        rospy.loginfo("Start grab")

                        # 发送抓取请求
                        self.grad_boxes()

                        rospy.loginfo("stand")
                        self.pose_twist.linear.z = 0.0
                        self.pose_pub.publish(self.pose_twist)
                        rospy.sleep(0.5)

                        # 返回
                        self.robot_moves_back()

                        rospy.spin()

    def getKey(self):
        """
        获取一个按键字符（非阻塞）

        此函数的目的是在不阻塞程序执行的情况下，检查是否有键盘输入。
        如果有输入，就读取一个字符；如果没有输入，则返回空字符串。
        这对于需要实时响应用户输入，同时不能中断程序正常执行的应用场景非常有用。
        """
        # 获取标准输入的文件描述符
        fd = sys.stdin.fileno()
        # 保存当前终端的设置
        old_settings = termios.tcgetattr(fd)
        try:
            # 将终端设置为原始模式，以便直接读取按键
            tty.setraw(sys.stdin.fileno())
            # 使用select.select函数监听标准输入，设置超时时间为0.1秒，实现非阻塞
            i, o, e = select.select([sys.stdin], [], [], 0.1)
            if i:
                # 如果有输入可用，读取一个字符
                key = sys.stdin.read(1)
            else:
                # 如果没有输入，返回空字符串
                key = ''
        finally:
            # 无论是否有输入，都恢复终端的原始设置
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        # 返回读取到的字符或空字符串
        return key

    def run(self):
        """
        主循环函数，用于读取键盘输入并发布消息以控制机器人。
        """
        # 初始化循环频率为30Hz
        rate = rospy.Rate(30)  # 20Hz
        # 提示用户使用键盘控制机器人的方法
        rospy.loginfo("Reading from keyboard...")
        rospy.loginfo("use space to enable/disable robot movement. Press Ctrl+C to quit.")

        # 主循环，直到节点被关闭
        while not rospy.is_shutdown():
            # 获取键盘输入
            key = self.getKey()
            # 如果有按键输入，则发布该键值
            if key:
                self.key_pub.publish(key)

            # 将头部转动的角度发布
            joint_data = [-12, 20]
            self.head_pub.publish(joint_data)

            # 发布joystick的消息
            self.joy_pub.publish(self.joy_msg)

            # 调用函数使机器人移动
            self.robot_moves_go()

            # 如果按下Ctrl-C，则退出循环
            if (key == '\x03'):  # Ctrl-C
                break

            # 按照初始化的频率休眠，以保持循环频率
            rate.sleep()


if __name__ == '__main__':
    try:
        node = MoveBoxes()
        node.run()
    except rospy.ROSInterruptException:
        cv2.destroyAllWindows()
        pass
