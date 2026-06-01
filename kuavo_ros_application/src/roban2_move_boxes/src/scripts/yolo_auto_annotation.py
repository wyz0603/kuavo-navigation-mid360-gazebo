#!/usr/bin/env python3
# -*- coding: utf-8 -*-


import cv2
from ultralytics import YOLO
from pathlib import Path
import datetime
from queue import Queue, Empty

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class YoloAutoAnnotation:
    def __init__(self):
        # 从ROS参数服务器获取参数
        rospy.init_node('yolo_auto_annotation', anonymous=True)

        self.model_path = rospy.get_param('~model_path', )
        self.base_dir = rospy.get_param('~base_dir')
        self.yolo_conf = rospy.get_param('~yolo_conf')

        rospy.loginfo(f"Using model: {self.model_path}")
        print(f"Using base_dir: {self.base_dir}")

        # 初始化YOLO模型
        self.model = YOLO(self.model_path)
        self.class_names = self.model.names

        # 创建保存目录
        self.images_dir = f"{self.base_dir}/images"
        self.labels_dir = f"{self.base_dir}/labels"
        Path(self.images_dir).mkdir(parents=True, exist_ok=True)
        Path(self.labels_dir).mkdir(parents=True, exist_ok=True)

        # 生成classes.txt文件
        classes_file = f"{self.base_dir}/labels/classes.txt"
        with open(classes_file, 'w') as f:
            for class_id, class_name in self.class_names.items():
                f.write(f"{class_name}\n")
        print(f"已生成类别文件: {classes_file} ({len(self.class_names)}个类别)")

        # 初始化ROS节点和订阅者
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.image_callback)

        # 图像队列用于线程间通信
        self.frame_queue = Queue(maxsize=10)

        rospy.loginfo(f"YOLO实时检测中... 等待图像数据 (模型: {self.model_path})")

    def image_callback(self, msg):
        try:
            # 将ROS图像消息转换为OpenCV图像格式
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            # 进行YOLO检测
            results = self.model.predict(frame, conf=self.yolo_conf, verbose=False)

            # 获取第一个结果（因为只处理单帧）
            if len(results) > 0:
                result = results[0]

                # 绘制检测结果
                annotated_frame = result.plot()

                # 如果队列满，则丢弃旧帧，保留最新帧
                if self.frame_queue.full():
                    self.frame_queue.get_nowait()
                self.frame_queue.put((annotated_frame, frame, results))
            else:
                if self.frame_queue.full():
                    self.frame_queue.get_nowait()
                self.frame_queue.put((frame.copy(), frame, None))

        except Exception as e:
            print(e)

    def run(self):
        try:
            while not rospy.is_shutdown():
                try:
                    annotated_frame, original_frame, results = self.frame_queue.get(timeout=1)
                    # 显示带标注的图像
                    try:
                        cv2.imshow(f'YOLO Detection ({self.model.overrides["model"]}) - Press SPACE to Save',
                                   annotated_frame)
                    except Exception as e:
                        print(e)
                    key = cv2.waitKey(30)

                    # 按空格键保存结果
                    if key % 256 == 32:  # 空格键保存结果
                        save_annotation(original_frame, results, self.images_dir, self.labels_dir)

                except Empty:
                    continue  # 队列为空时继续等待
        finally:
            cv2.destroyAllWindows()


def save_annotation(frame, results, images_dir, labels_dir):
    """保存当前帧的图像和标注"""
    # 生成唯一文件名
    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    img_name = f"{images_dir}/{timestamp}.jpg"
    txt_name = f"{labels_dir}/{timestamp}.txt"

    # 保存原始图像
    cv2.imwrite(img_name, frame)
    print(f"保存图像: {img_name}")

    # 获取检测框信息（归一化坐标）
    if len(results) > 0 and hasattr(results[0], 'boxes'):
        result = results[0]
        boxes = result.boxes.xywhn.cpu().numpy()  # 归一化的(x_center, y_center, width, height)
        class_ids = result.boxes.cls.cpu().numpy().astype(int)

        # 写入标注文件
        with open(txt_name, 'w') as f:
            for i, box in enumerate(boxes):
                class_id = class_ids[i]
                # YOLO格式: class_id x_center y_center width height
                f.write(f"{class_id} {box[0]:.6f} {box[1]:.6f} {box[2]:.6f} {box[3]:.6f}\n")

        print(f"保存标注: {txt_name} (包含 {len(boxes)} 个对象)")
    else:
        # 没有检测到任何对象时创建空标注文件
        open(txt_name, 'w').close()
        print(f"保存空标注: {txt_name} (未检测到对象)")


if __name__ == "__main__":
    yolo_annotator = YoloAutoAnnotation()
    yolo_annotator.run()
