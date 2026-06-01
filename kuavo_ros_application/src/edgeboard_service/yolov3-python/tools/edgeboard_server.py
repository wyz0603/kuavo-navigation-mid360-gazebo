#!/usr/bin/env python

from __future__ import print_function
from edgeboard_service.srv import EbMessage,EbMessageResponse #注意是功能包名.srv

import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError

import json
import os
import argparse
import sys
import yaml
import numpy as np
import warnings
#import cv2
warnings.filterwarnings("ignore")

parent_path = os.path.abspath(os.path.join(__file__, *([".."] * 2)))
sys.path.insert(0, parent_path)
from yolo import YoloV3, vis, labels

os.chdir(parent_path)

# 解析命令行参数，配置模型参数和优化器等参数
def argsparser():

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--config",
        type=str,
        default="./model/config.json",
        help=("path of deploy config.json"),
    )
    parser.add_argument(
        "--infer_yml",
        type=str,
        default="./model/infer_cfg.yml",
        help=("path of infer_cfg.yml"),
    )
    parser.add_argument(
        "--visualize", action="store_true", help="whether to visualize."
    )
    parser.add_argument(
        "--with_profile", action="store_true", help="whether to predict with profile."
    )
    return parser

# 获取检测结果（带坐标，打印到终端）
def labels_and_xy(results, threshold=0.3):
    for i in range(0, len(results), 6):
        label = int(results[i])
        score = results[i + 1]
        if score < threshold:
            continue
        #xmin+xmax
        xmid = (int(results[i + 2])+int(results[i + 4]))/2
        #ymin+ymax
        ymid = (int(results[i + 3])+int(results[i + 5]))/2

        rospy.loginfo(str(labels[label]) + " " + str(score)[:4] + " " + str(xmid)[:3] + " " + str(ymid)[:3])

# 获取检测结果（不带坐标，将信息整合成字符串发给上位机）
# 注：经多次测试，模型的result会将同类检测结果逐个输出后再输出其他类检测结果
# 依靠这个特性进行处理，可以有效减轻算法时空间复杂度
def get_str_results(results, threshold=0.3):
    str_results=""
    label_last = -1
    # 标志位 第几个标签
    count_flag = 0
    # 十行两列数组 即最多保存十类检测结果
    num_results = np.zeros((11,2))
    # 第一次循环 收集标签和对应数量
    for i in range(0, len(results), 6):
        label = int(results[i])
        score = results[i + 1]
        # 置信度过低时忽略
        if score < threshold:
            continue
        # 保存本次结果
        if label==label_last:
            num_results[count_flag][1]+=1
        else :
            label_last=label
            count_flag+=1
            num_results[count_flag][0]=label
            #print(label)
            num_results[count_flag][1]+=1
    # 第二次循环 将结果整合成一个字符串
    for i in range(1,count_flag+1):
        str_results += str(int(num_results[i][0])) + " " + str(int(num_results[i][1])) + " "
    
    return str_results

# 将结点定义为类
class SubscriberNode:
    def __init__(self,detector,with_profile,visualize):
        # 启动ros结点 启动服务
        rospy.init_node('edgeboard_server_lite', anonymous=True)
        self.server = rospy.Service('edgeboard_yolo', EbMessage, self.edgeboard_callback)
        # 启动话题发布者
        self.image_pub = rospy.Publisher('/image_view/image_raw', Image, queue_size = 1)
        print("Ready to edgeboard yolo.")
        # 参数配置
        self.detector=detector
        self.with_profile=with_profile
        self.visualize=visualize
        
        # 使用一个字符串保存图像处理结果
        self.str_results = "results init"
        
        # 设置频率
        self.rate = rospy.Rate(1) # 1Hz

    def subscriber_init(self):
        # 创建一个Subscriber并订阅'topic'
        # 上位机的摄像头服务
        self.subscriber = rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback)
        # ros自带的摄像头驱动
        #self.subscriber = rospy.Subscriber('/usb_cam/image_raw', Image, self.image_callback)
        
        # 使用一个计数器来控制何时取消订阅
        self.counter = 0
        self.counter_flag = 1

    # 图像话题回调函数
    def image_callback(self, data):

        # 处理接收到的数据
        #rospy.loginfo("Received: %s", data.data)

        # 生成CvBridge实例
        bridge = CvBridge()

        # 提取图像话题
        try:
            # 将ROS图像消息转换为OpenCV格式
            cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))
        
        # 处理图像
        if self.with_profile:
            results = self.detector.predict_profile(cv_image)
            print("total time: ", self.detector.total_time)
            labels_and_xy(results)
        else:
            results = self.detector.predict_image(cv_image)
        
        # 保存图像处理结果
        self.str_results = get_str_results(results)

        # 显示图像并保存
        if self.visualize:
            render_img = vis(cv_image, results)
            # 保存到本地
            cv2.imwrite("./vis.jpg", render_img)
            # 发布话题
            self.image_pub.publish(bridge.cv2_to_imgmsg(render_img, "bgr8")) #发布消息
            # 若不接显示屏，需要注释下面这行
            #cv2.imshow("Camera Image", render_img)
            
        rospy.loginfo("test success")
        cv2.waitKey(1)
        
        self.counter += 1
        # 设置条件取消订阅
        print("callback:",self.counter)
        if self.counter >= self.counter_flag: # 例如：接收5条消息s后取消订阅
            self.unsubscribe()

    # 服务回调函数
    def edgeboard_callback(self, data):

        print("Accept config:",data.config)
        # 开始订阅图像信息 等待处理结果
        self.subscriber_init()
        self.run()
        
        print("Return result:", self.str_results )
        return EbMessageResponse( self.str_results )

    # 取消订阅话题
    def unsubscribe(self):
        self.subscriber.unregister()
        rospy.loginfo("Unsubscribed from topic.")

    def run(self):
        # 任务完成后退出订阅等待循环
        while not (rospy.is_shutdown() or (self.counter >= self.counter_flag)):
            # 循环保持节点活着
            print("while:",self.counter,' ',rospy.is_shutdown())
            self.rate.sleep()
        # 退出循环
        self.counter = 0
        

def main(args):
    # 模型参数 使用默认配置即可
    config = args.config
    assert os.path.exists(config), "deploy_config does not exist."
    
    infer_yml = args.infer_yml
    assert os.path.exists(infer_yml), "infer_yml does not exist."
    with open(infer_yml, "r") as f:
        infer_yml = yaml.safe_load(f)

    # 参数配置
    detector = YoloV3(config, infer_yml)
    with_profile = args.with_profile
    visualize = args.visualize

    # 启动ros结点
    #rospy.init_node('edgeboard_server')
    
    # 初始化结点服务
    node = SubscriberNode( detector , with_profile , visualize )

    rospy.spin()


if __name__ == "__main__":
    parser = argsparser()
    #args = parser.parse_args()
    args = parser.parse_args(rospy.myargv(argv=sys.argv)[1:])
    main(args)
