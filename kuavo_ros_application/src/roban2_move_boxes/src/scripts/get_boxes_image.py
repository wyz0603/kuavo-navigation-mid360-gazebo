#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import cv2
import os
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import threading


def save_images_from_camera(output_folder):
    """
    从ROS图像话题捕获数据，按下空格键时保存当前帧为图像。

    :param output_folder: 保存图像的文件夹路径
    """
    # 检查输出文件夹是否存在，不存在则创建
    output_folder = os.path.expanduser(output_folder)
    if not os.path.exists(output_folder):
        rospy.loginfo(f"输出目录 {output_folder} 不存在，正在创建...")
        os.makedirs(output_folder)
    else:
        rospy.loginfo(f"输出目录 {output_folder} 已存在。")

    # 创建CvBridge对象
    bridge = CvBridge()

    # 共享资源：用于存储最新图像帧
    latest_frame = None
    frame_lock = threading.Lock()
    save_count = 0

    # 图像回调函数（仅更新最新帧）
    def image_callback(msg):
        nonlocal latest_frame
        try:
            # 将ROS图像消息转换为OpenCV图像
            frame = bridge.imgmsg_to_cv2(msg, "bgr8")
            with frame_lock:
                latest_frame = frame.copy()
        except Exception as e:
            rospy.logerr(e)

    # 订阅图像话题
    image_topic = "/camera/color/image_raw"  # 根据实际发布的图像话题修改此值
    rospy.Subscriber(image_topic, Image, image_callback)

    # 主线程负责显示图像和处理按键事件
    rospy.loginfo(f"正在监听图像话题，请确保摄像头节点已启动并发布图像数据...")

    while not rospy.is_shutdown():
        frame = None
        with frame_lock:
            if latest_frame is not None:
                frame = latest_frame.copy()

        if frame is not None:
            try:
                cv2.imshow('Camera Feed', frame)
            except Exception as e:
                rospy.logerr(e)

            # # 在主线程中调用 waitKey()
            key = cv2.waitKey(30)

            # 按下空格键保存当前帧
            if key == 32:  # 32 是空格键的 ASCII 码
                timestamp = int(time.time())
                image_path = os.path.join(output_folder, f"frame_{timestamp}_{save_count:04d}.jpg")
                cv2.imwrite(image_path, frame)
                rospy.loginfo(f"已保存图像到 {image_path}")
                save_count += 1

            # 按下 'q' 键退出循环
            elif key == ord('q'):
                break

    # 关闭所有窗口
    cv2.destroyAllWindows()


if __name__ == "__main__":
    # 初始化ROS节点
    rospy.init_node('image_saver', anonymous=True)

    # 从ROS参数服务器获取输出路径，默认值为 '~/Desktop/camera_images'
    output_folder = rospy.get_param('~output_folder', '~/Desktop/camera_images')
    rospy.loginfo(f"图像将保存到目录: {output_folder}")

    save_images_from_camera(
        output_folder=output_folder
    )