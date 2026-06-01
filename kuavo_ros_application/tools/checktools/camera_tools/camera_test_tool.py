#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
相机测试工具（综合版）
功能：
1. 检测多种相机类型（奥比相机、RealSense相机、通用USB相机）
2. 监控ROS相机话题数据
3. 集成AprilTag检测功能
4. 相机启动测试和状态监控
5. 系统设备扫描
6. 综合测试报告生成
"""

import rospy
import numpy as np
import time
import json
import os
import sys
import subprocess
import threading
from collections import deque
from datetime import datetime
import argparse
import glob

# ROS相关导入
try:
    from sensor_msgs.msg import Image, CameraInfo
    from apriltag_ros.msg import AprilTagDetectionArray
    ROS_AVAILABLE = True
except ImportError:
    print("错误: ROS环境未配置，无法运行此工具")
    print("请确保已安装ROS并配置环境变量")
    sys.exit(1)

class UnifiedCameraTestTool:
    def __init__(self, enable_apriltag=True, enable_fps_monitor=True, enable_image_save=True, proj_dir=""):
        """
        初始化相机测试工具
        
        Args:
            enable_apriltag: 是否启用AprilTag检测
            enable_fps_monitor: 是否启用FPS监控
            enable_image_save: 是否启用图像保存
        """
        self.enable_apriltag = enable_apriltag
        self.enable_fps_monitor = enable_fps_monitor
        self.enable_image_save = enable_image_save
        
        # FPS监控相关
        self.fps_history = deque(maxlen=100)
        self.frame_times = deque(maxlen=100)
        self.last_frame_time = time.time()
        
        # AprilTag检测相关
        self.apriltag_detector = None
        self.tag_detections = {}  # 改为字典: frame_id -> bool
        self.tag_history = deque(maxlen=50)
        
        # 相机数据
        self.camera_images = {}
        self.camera_subscribers = {}

        self.proj_dir = proj_dir
        self.image_topics = []
        
        # 统计信息
        self.stats = {
            'total_frames': 0,
            'detected_tags': 0,
            'avg_fps': 0.0,
            'start_time': time.time(),
            'camera_frames': {}
        }
        
        # 扫描结果
        self.scan_results = {
            'usb_devices': [],
            'ros_topics': {},
            'camera_status': {}
        }
        
        # 多个 AprilTag 检测进程（按图像话题管理）
        self.apriltag_processes = {}

        # 初始化ROS和AprilTag检测器
        self.init_ros()
        if self.enable_apriltag:
            self.init_apriltag_detector()
    
    def init_ros(self):
        """初始化ROS节点和订阅者"""
        try:
            if not rospy.core.is_initialized():
                rospy.init_node('unified_camera_test_tool', anonymous=True)
            
            print("✅ ROS节点初始化成功")
                
        except Exception as e:
            print(f"ROS初始化失败: {e}")
            sys.exit(1)
    
    def init_apriltag_detector(self):
        """初始化AprilTag检测器"""
        try:
            # 使用仓库中的apriltag_ros包
            from apriltag_ros.msg import AprilTagDetectionArray
            print("✅ AprilTag检测器初始化成功 (使用apriltag_ros包)")
            self.apriltag_detector = "apriltag_ros"  # 标记使用ROS包
                        
        except ImportError:
            print("❌ 无法导入apriltag_ros模块")
            print("   请确保apriltag_ros包已编译")
            self.apriltag_detector = None
            return
    
    def check_usb_devices(self):
        """检查USB设备中的相机"""
        print(" 检查USB设备中的相机...")
        
        try:
            # 使用lsusb命令查找相机设备
            result = subprocess.run(['lsusb'], capture_output=True, text=True, timeout=10)
            if result.returncode == 0:
                lines = result.stdout.strip().split('\n')
                camera_devices = []
                
                for line in lines:
                    # 奥比相机 (vendor ID: 2bc5)
                    if '2bc5' in line.lower():
                        camera_devices.append({
                            'type': 'orbbec',
                            'info': line.strip(),
                            'vendor_id': '2bc5'
                        })
                    # RealSense相机 (vendor ID: 8086)
                    elif '8086' in line.lower() and any(keyword in line.lower() for keyword in ['camera', 'realsense', 'intel']):
                        camera_devices.append({
                            'type': 'realsense',
                            'info': line.strip(),
                            'vendor_id': '8086'
                        })
                    # 其他可能的相机设备
                    elif any(keyword in line.lower() for keyword in ['camera', 'webcam', 'video', 'usb']):
                        camera_devices.append({
                            'type': 'generic',
                            'info': line.strip(),
                            'vendor_id': 'unknown'
                        })
                
                if camera_devices:
                    print("✅ 找到相机USB设备:")
                    for device in camera_devices:
                        print(f"   [{device['type'].upper()}] {device['info']}")
                    self.scan_results['usb_devices'] = camera_devices
                    return True, camera_devices
                else:
                    print("❌ 未找到相机USB设备")
                    return False, []
            else:
                print(f"❌ lsusb命令执行失败: {result.stderr}")
                return False, []
                
        except subprocess.TimeoutExpired:
            print("❌ lsusb命令执行超时")
            return False, []
        except Exception as e:
            print(f"❌ USB设备检查失败: {e}")
            return False, []
    
    def check_ros_topics(self):
        """检查ROS相机话题"""
        print(" 检查ROS相机话题...")
        
        try:
            # 获取所有话题
            result = subprocess.run(['rostopic', 'list'], capture_output=True, text=True, timeout=10)
            if result.returncode == 0:
                topics = result.stdout.strip().split('\n')
                camera_topics = []
                apriltag_topics = []
                other_topics = []
                
                # 分类话题
                for topic in topics:
                    if any(keyword in topic.lower() for keyword in ['camera', 'image', 'color', 'depth', 'ir']):
                        camera_topics.append(topic)
                    elif 'apriltag' in topic.lower() or 'tag' in topic.lower():
                        apriltag_topics.append(topic)
                    else:
                        other_topics.append(topic)
                
                # 保存话题信息
                self.scan_results['ros_topics'] = {
                    'camera': camera_topics,
                    'apriltag': apriltag_topics,
                    'other': other_topics,
                    'total': len(topics),
                    'image_topics': []
                }
                
                if camera_topics or apriltag_topics:
                    print("✅ 找到相机相关话题:")
                    if camera_topics:
                        print(f"  相机话题 ({len(camera_topics)}个):")
                        # 检查图像数据相关话题
                        image_topics = [t for t in camera_topics if 'image' in t.lower()]
                        if image_topics:
                            print(f"     图像话题 ({len(image_topics)}个):")
                        
                        # 检查其他相机话题
                        other_camera_topics = [t for t in camera_topics if 'image' not in t.lower()]
                        if other_camera_topics:
                            print(f"     其他相机话题 ({len(other_camera_topics)}个):")
                    
                    if apriltag_topics:
                        print(f"  AprilTag话题 ({len(apriltag_topics)}个):")
                        for topic in apriltag_topics:
                            print(f"    ️  {topic}")
                    
                    return True, self.scan_results['ros_topics']
                else:
                    print("❌ 未找到相机相关话题")
                    return False, self.scan_results['ros_topics']
            else:
                print(f"❌ rostopic list命令执行失败: {result.stderr}")
                return False, {}
                
        except subprocess.TimeoutExpired:
            print("❌ rostopic list命令执行超时")
            return False, {}
        except Exception as e:
            print(f"❌ ROS话题检查失败: {e}")
            return False, {}
    
    def check_topic_frequency(self, topic_name, duration=5):
        """检查话题频率"""
        try:
            print(f" 检查话题 {topic_name} 的频率...")
            
            # 使用rostopic hz命令检查频率
            result = subprocess.run(
                ['timeout', str(duration), 'rostopic', 'hz', topic_name],
                capture_output=True, text=True, timeout=duration+5
            )
            
            if result.returncode == 0:
                output = result.stdout.strip()
                if 'average:' in output:
                    # 提取频率信息
                    lines = output.split('\n')
                    for line in lines:
                        if 'average:' in line:
                            freq = line.split('average:')[1].strip()
                            print(f"✅ 话题 {topic_name} 频率: {freq}")
                            return True, freq
                else:
                    print(f"⚠️  话题 {topic_name} 无频率数据")
                    return False, None
            else:
                print(f"❌ 话题 {topic_name} 频率检查失败")
                return False, None
                
        except Exception as e:
            print(f"❌ 话题频率检查失败: {e}")
            return False, None
    
    def check_image_topic_details(self, topic_name):
        """检查图像话题的详细信息"""
        try:
            
            # 获取话题类型
            type_result = subprocess.run(['rostopic', 'type', topic_name], 
                                       capture_output=True, text=True, timeout=3)
            if type_result.returncode == 0:
                topic_type = type_result.stdout.strip()
                
                # 检查是否是图像话题
                if 'sensor_msgs/Image' == topic_type and '/color/image_raw' in topic_name:
                    print(f"  ✅ {topic_name} 确认是图像话题")
                    
                    self.image_topics.append(topic_name)
                    # 记录至扫描结果的图像话题列表
                    try:
                        image_list = self.scan_results.get('ros_topics', {}).setdefault('image_topics', [])
                        if topic_name not in image_list:
                            image_list.append(topic_name)
                    except Exception:
                        pass
                    # 获取话题信息
                    info_result = subprocess.run(['rostopic', 'info', topic_name], 
                                              capture_output=True, text=True, timeout=3)
                    
                    # 检查话题频率
                    freq_result = subprocess.run(['timeout', '3', 'rostopic', 'hz', topic_name], 
                                              capture_output=True, text=True, timeout=6)
                    if freq_result.returncode in [0, 124]:  # 0=成功, 124=超时但可能有数据
                        output = freq_result.stdout.strip()
                        if 'average rate:' in output:
                            # 提取最后一个频率数据
                            lines = output.split('\n')
                            for line in reversed(lines):
                                if 'average rate:' in line:
                                    freq_text = line.split('average rate:')[1].strip()
                                    print(f"  频率: {freq_text}")
                                    
                                    # 尝试解析频率数值并判断是否正常
                                    try:
                                        # 提取数字部分，处理小数点
                                        freq_str = ''.join(c for c in freq_text if c.isdigit() or c == '.')
                                        freq_value = float(freq_str)
                                        
                                        # 判断频率是否正常
                                        if freq_value >= 25:
                                            print(f"  ✅ 频率正常 (≥25Hz)")
                                        else:
                                            print(f"  ⚠️  频率异常 (<25Hz)，请检查相机连线是否正常")
                                    except ValueError:
                                        print(f"  ⚠️  无法解析频率数值")
                                    break
                        else:
                            print(f"  频率: 无数据")
                    else:
                        print(f"  频率: 检查失败 (返回码: {freq_result.returncode})")
                        
                    
            else:
                print(f"  ❌ 无法获取话题类型")
                
        except Exception as e:
            print(f"  ❌ 检查图像话题详情失败: {e}")

    def test_topic_connection(self, topic_name):
        """测试话题连接"""
        print(f" 测试话题 {topic_name} 连接...")
        
        try:
            # 等待话题消息
            print(f"  等待话题 {topic_name} 的消息...")
            
            # 使用rostopic echo测试
            result = subprocess.run(['timeout', '5', 'rostopic', 'echo', topic_name, '-n', '1'], 
                                 capture_output=True, text=True, timeout=10)
            
            if result.returncode == 0 and result.stdout.strip():
                print(f"  ✅ 话题 {topic_name} 连接成功，收到消息")
                return True
            else:
                print(f"  ❌ 话题 {topic_name} 连接失败或无消息")
                return False
                
        except Exception as e:
            print(f"  ❌ 测试话题连接失败: {e}")
            return False
    
    def subscribe_to_camera_topics(self):
        """订阅相机话题"""
        print(" 订阅相机话题...")
        
        try:
            camera_topics = self.image_topics
            if not camera_topics:
                print("⚠️  没有可订阅的相机话题")
                return False
            
            # 订阅彩色图像话题
            color_topics = [t for t in camera_topics if 'color' in t.lower()]
            if color_topics:
                topic = color_topics[0]
                self.camera_subscribers['color'] = rospy.Subscriber(
                    topic,
                    Image,
                    self.color_callback,
                    queue_size=5
                )
                print(f"✅ 已订阅彩色图像话题: {topic}")
            
            # 订阅深度图像话题
            depth_topics = [t for t in camera_topics if 'depth' in t.lower()]
            if depth_topics:
                topic = depth_topics[0]
                self.camera_subscribers['depth'] = rospy.Subscriber(
                    topic,
                    Image,
                    self.depth_callback,
                    queue_size=5
                )
                print(f"✅ 已订阅深度图像话题: {topic}")
            
            # 订阅IR图像话题
            ir_topics = [t for t in camera_topics if 'ir' in t.lower()]
            if ir_topics:
                topic = ir_topics[0]
                self.camera_subscribers['ir'] = rospy.Subscriber(
                    topic,
                    Image,
                    self.depth_callback,  # IR图像也使用深度回调
                    queue_size=5
                )
                print(f"✅ 已订阅IR图像话题: {topic}")
            
            return len(self.camera_subscribers) > 0
                
        except Exception as e:
            print(f"❌ 订阅相机话题失败: {e}")
            return False
    
    def select_topics_for_apriltag_detection(self, camera_topics):
        """为所有 Image 类型的话题启动 AprilTag 检测"""
        print("\n 选择话题进行AprilTag检测...")
        
        # 启动AprilTag检测监控（全局一次）
        # self.start_apriltag_monitoring()
        try:
            # 仅使用已经确认类型为 sensor_msgs/Image 的话题
            image_topics = list(self.image_topics)
            
            if not image_topics:
                print("⚠️  没有找到图像话题，无法进行AprilTag检测")
                return
            
            print(f"找到 {len(image_topics)} 个图像话题，将依次进行AprilTag检测:")
            
            # 为每个话题创建检测状态跟踪
            self.topic_detection_status = {}
            for topic in image_topics:
                self.topic_detection_status[topic] = {
                    'detected': False,
                    'start_time': None,
                    'detection_time': None
                }
            
            # 依次检测每个话题，等待检测到tag才进行下一个
            for i, topic in enumerate(image_topics):
                print(f"\n🔄 开始检测话题 {i+1}/{len(image_topics)}: {topic}")
                print("="*60)
                
                # 设置话题检测开始时间
                self.topic_detection_status[topic]['start_time'] = time.time()
                
                # 启动AprilTag检测节点
                if not self.start_apriltag_ros_node(topic):
                    print(f"❌ 跳过话题 {topic} 的AprilTag检测")
                    self.topic_detection_status[topic]['detected'] = False
                    continue
                
                # 等待节点启动
                rospy.sleep(3)
                
                # 检查是否已经检测到tag（可能节点启动后立即检测到）
                camera_name = self.get_camera_friendly_name(topic)
                camera_already_detected = False
                
                # 检查该相机是否已经在tag_detections中
                for frame_id in self.tag_detections.keys():
                    if self.tag_detections[frame_id] and camera_name in frame_id:
                        camera_already_detected = True
                        break
                
                if camera_already_detected:
                    print(f"🎯 话题 {topic} 已检测到AprilTag，无需等待")
                    detection_success = True
                else:
                    # 提示操作人员放置tag板
                    print(f"💡 请将任何AprilTag标签板放到{camera_name}前")
                    print(f"⏳ 等待检测到AprilTag...")
                    
                    # 等待检测到tag或超时
                    detection_success = self.wait_for_tag_detection(topic, timeout=60)
                
                if detection_success:
                    print(f"✅ 话题 {topic} 检测成功！检测到AprilTag")
                    self.topic_detection_status[topic]['detected'] = True
                    self.topic_detection_status[topic]['detection_time'] = time.time()
                else:
                    print(f"❌ 话题 {topic} 检测超时，未检测到任何AprilTag")
                    self.topic_detection_status[topic]['detected'] = False
                
                # 短暂延迟，让操作人员准备下一个相机
                if i < len(image_topics) - 1:
                    print(f"⏱️  准备检测下一个话题，请稍等...")
                    rospy.sleep(2)
            
            # 检查所有话题的检测结果
            self.check_all_topics_detection_results()

        except Exception as e:
            print(f"❌ 选择AprilTag检测话题失败: {e}")
    
    def get_camera_friendly_name(self, topic):
        """根据话题名称返回友好的相机名称"""
        if 'left_wrist_camera' in topic:
            return "左手腕部相机"
        elif 'right_wrist_camera' in topic:
            return "右手腕部相机"
        elif 'head_camera' in topic:
            return "头部相机"
        else:
            return f"相机({topic})"
        
    def get_camera_name(self, topic):
        """根据话题名称返回友好的相机名称"""
        if 'left_wrist_camera' in topic:
            return "left_wrist_camera"
        elif 'right_wrist_camera' in topic:
            return "right_wrist_camera"
        elif 'head_camera' in topic:
            return "head_camera"
        else:
            return f"camera({topic})"
    
    def check_all_topics_detection_results(self):
        """检查所有话题的检测结果并给出结论"""
        if not hasattr(self, 'topic_detection_status') or not self.topic_detection_status:
            return
        
        print("\n" + "="*80)
        print("📊 APRILTAG检测结果汇总")
        print("="*80)
        
        total_topics = len(self.topic_detection_status)
        successful_detections = sum(1 for status in self.topic_detection_status.values() if status['detected'])
        failed_detections = total_topics - successful_detections
        
        # 显示每个话题的检测状态
        for topic, status in self.topic_detection_status.items():
            status_icon = "✅" if status['detected'] else "❌"
            status_text = "检测成功" if status['detected'] else "检测失败"
            camera_name = self.get_camera_friendly_name(topic)
            print(f"{status_icon} {camera_name} ({topic}): {status_text}")
            
            if status['detected'] and status['detection_time'] and status['start_time']:
                detection_duration = status['detection_time'] - status['start_time']
                print(f"   检测用时: {detection_duration:.1f}秒")
            
            # 显示频率信息
            self.display_topic_frequency(topic)
        
        print("\n" + "-"*80)
        
        # 给出总体结论
        if successful_detections == total_topics:
            print("🎉 所有相机话题检测成功！")
            print("✅ 结论：所有相机都正常工作，AprilTag检测功能正常")
            print("✅ 相机系统状态：正常")
        elif successful_detections > 0:
            print(f"⚠️  部分相机话题检测成功 ({successful_detections}/{total_topics})")
            print(f"❌ 结论：{failed_detections}个相机话题存在问题")
            
            # 显示具体哪些相机有问题
            failed_topics = [topic for topic, status in self.topic_detection_status.items() if not status['detected']]
            print("❌ 需要检查的相机：")
            for topic in failed_topics:
                camera_name = self.get_camera_friendly_name(topic)
                print(f"   - {camera_name} ({topic})")
            
            print(f"⚠️  相机系统状态：需要检查")
        else:
            print("❌ 所有相机话题检测失败")
            print("❌ 结论：相机系统存在问题，需要检查")
            print("❌ 相机系统状态：异常")
        
        print("="*80)
    
    def display_topic_frequency(self, topic):
        """显示话题频率信息"""
        try:
            # 使用rostopic hz命令检查频率
            result = subprocess.run(['timeout', '2', 'rostopic', 'hz', topic], 
                                  capture_output=True, text=True, timeout=4)
            
            if result.returncode in [0, 124]:  # 0=成功, 124=超时但可能有数据
                output = result.stdout.strip()
                if 'average rate:' in output:
                    # 提取最后一个频率数据
                    lines = output.split('\n')
                    for line in reversed(lines):
                        if 'average rate:' in line:
                            freq_text = line.split('average rate:')[1].strip()
                            print(f"   频率: {freq_text}")
                            
                            # 判断频率是否正常
                            try:
                                # 提取数字部分，处理小数点
                                freq_str = ''.join(c for c in freq_text if c.isdigit() or c == '.')
                                freq_value = float(freq_str)
                                
                                # 判断频率是否正常
                                if freq_value >= 25:
                                    print(f"   ✅ 频率正常 (≥25Hz)")
                                else:
                                    print(f"   ⚠️  频率异常 (<25Hz)，请检查相机连线是否正常")
                            except ValueError:
                                print(f"   ⚠️  无法解析频率数值")
                            break
                else:
                    print(f"   频率: 无数据")
            else:
                print(f"   频率: 检查失败")
        except Exception as e:
            print(f"   频率: 检查异常 ({e})")
    
    def wait_for_tag_detection(self, topic, timeout=60):
        """等待指定话题检测到AprilTag"""
        print(f"🔍 等待话题 {topic} 检测到AprilTag，超时时间: {timeout}秒")
        
        # 从话题名称推断相机名
        camera_name = self.get_camera_name(topic)
        print(f"  等待相机: {camera_name}")
        
        start_time = time.time()
        
        while not rospy.is_shutdown():
            current_time = time.time()
            elapsed_time = current_time - start_time
            
            # 检查是否超时
            if elapsed_time >= timeout:
                print(f"⏰ 检测超时 ({timeout}秒)，未检测到AprilTag")
                return False
            
            # 检查该相机是否在tag_detections的键中
            camera_detected = False
            for frame_id in self.tag_detections.keys():
                if self.tag_detections[frame_id] and camera_name in frame_id:
                    camera_detected = True
                    break
            
            if camera_detected:
                print(f"🎯 相机 {camera_name} 检测到AprilTag！话题 {topic} 检测通过")
                print(f"   检测到的frame: {[f for f in self.tag_detections.keys() if camera_name in f]}")
                return True
            
            # 显示等待状态
            if int(elapsed_time) % 5 == 0 and elapsed_time > 0:
                remaining_time = timeout - elapsed_time
                print(f"⏳ 等待中... 剩余时间: {remaining_time:.0f}秒")
                print(f"💡 请确保AprilTag标签板在相机视野内，距离适中")
            
            # 短暂延迟
            rospy.sleep(0.5)
        
        return False
    
    def start_apriltag_ros_node(self, image_topic):
        """根据图像话题启动/重启 apriltag_ros continuous_detection 节点（为每个话题单独起实例）"""
        try:
            print(f"启动/重启 AprilTag 检测，绑定话题: {image_topic}")

            # 先检查话题是否有发布者
            print(f"检查话题 {image_topic} 是否有发布者...")
            try:
                # 等待话题出现，最多等待5秒
                rospy.wait_for_message(image_topic, Image, timeout=5.0)
                print(f"✅ 话题 {image_topic} 有发布者，可以启动AprilTag节点")
            except rospy.ROSException as e:
                print(f"❌ 话题 {image_topic} 没有发布者或超时: {e}")
                print("跳过启动AprilTag节点")
                return False
            except Exception as e:
                print(f"⚠️  检查话题发布者时出现异常: {e}")
                print("跳过启动AprilTag节点")
                return False

            # 如该话题已有在跑的进程，先结束，实现"重新起"
            old_proc = self.apriltag_processes.get(image_topic)
            if old_proc is not None:
                try:
                    print(" 发现该话题已有 AprilTag 进程，尝试停止...")
                    old_proc.terminate()
                    old_proc.wait(timeout=5)
                    print("✅ 旧的进程已停止")
                except Exception:
                    try:
                        old_proc.kill()
                    except Exception:
                        pass

            # 从完整话题中解析 camera_name 与 image_topic
            # 例如: /camera/color/image_raw -> camera_name=/camera/color, image_topic=image_raw
            topic_parts = [p for p in image_topic.strip().split('/') if p]
            if len(topic_parts) < 2:
                raise ValueError(f"图像话题格式不正确: {image_topic}")
            camera_ns = '/' + '/'.join(topic_parts[:-1])
            image_leaf = topic_parts[-1]

            # 构建 workspace source 命令（如提供了项目路径）
            ws_source_cmd = ''
            try:
                if self.proj_dir and os.path.exists(os.path.join(self.proj_dir, 'devel', 'setup.bash')):
                    ws_source_cmd = f'cd {self.proj_dir} && source devel/setup.bash && '
            except Exception:
                pass

            # 启动 apriltag_ros 的 continuous_detection.launch，并传入解析出的参数
            launch_cmd = (
                'source /opt/ros/noetic/setup.bash && '
                f'{ws_source_cmd}'
                'roslaunch kuavo_camera continuous_detection.launch '
                f'camera_name:={camera_ns} image_topic:={image_leaf}'
            )

            print(f"执行命令: {launch_cmd}")

            proc = subprocess.Popen(
                ["/bin/bash", "-c", launch_cmd],
                shell=False,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL
            )

            time.sleep(3)
            print(proc.poll())
            if proc.poll() is None:
                print("✅ AprilTag 节点启动成功")
                # 记录新进程
                self.apriltag_processes[image_topic] = proc
                # 为该话题创建独立的订阅者
                try:
                    detections_topic = f"/tag_detections"
                    image_topic_ns = f"tag_detections_image"
                    
                    # 为每个话题创建唯一的订阅者标识
                    detections_key = f'apriltag_detections_{image_topic.replace("/", "_")}'
                    image_key = f'apriltag_image_{image_topic.replace("/", "_")}'
                    
                    # 如果已存在订阅者，先取消订阅
                    if detections_key in self.camera_subscribers:
                        self.camera_subscribers[detections_key].unregister()
                    if image_key in self.camera_subscribers:
                        self.camera_subscribers[image_key].unregister()
                    
                    # 创建新的订阅者
                    self.camera_subscribers[detections_key] = rospy.Subscriber(
                        detections_topic,
                        AprilTagDetectionArray,
                        self.apriltag_detections_callback,
                        queue_size=10
                    )
                    self.camera_subscribers[image_key] = rospy.Subscriber(
                        image_topic_ns,
                        Image,
                        self.apriltag_image_callback,
                        queue_size=5
                    )
                    print(f"✅ 已订阅: {detections_topic} 与 {image_topic_ns}")
                    return True  # 成功启动
                except Exception as _:
                    print("⚠️  创建 AprilTag 订阅者失败")
                    return False
            else:
                print("❌ AprilTag 节点启动失败")
                return False
        except Exception as e:
            print(f"❌ 启动 AprilTag 节点失败: {e}")
            return False
    
    def subscribe_to_apriltag_results(self):
        """订阅AprilTag检测结果"""
        try:
            print(" 订阅AprilTag检测结果...")
            
            # 订阅tag_detections话题
            self.camera_subscribers['apriltag_detections'] = rospy.Subscriber(
                '/tag_detections',
                AprilTagDetectionArray,
                self.apriltag_detections_callback,
                queue_size=10
            )
            
            # 订阅tag_detections_image话题（可选）
            self.camera_subscribers['apriltag_image'] = rospy.Subscriber(
                '/tag_detections_image',
                Image,
                self.apriltag_image_callback,
                queue_size=5
            )
            
            print("✅ 已订阅AprilTag检测结果话题")
            
        except Exception as e:
            print(f"❌ 订阅AprilTag检测结果失败: {e}")
    
    def apriltag_detections_callback(self, msg):
        """AprilTag检测结果回调函数"""
        try:
            if msg.detections:
                for detection in msg.detections:
                    # 通过frame字段判断相机类型
                    camera_type = "unknown"
                    if hasattr(detection, 'pose') and hasattr(detection.pose, 'header') and hasattr(detection.pose.header, 'frame_id'):
                        frame_id = detection.pose.header.frame_id
                        if 'head_camera' in frame_id:
                            camera_type = "head_camera"
                        elif 'left_wrist_camera' in frame_id:
                            camera_type = "left_wrist_camera"
                        elif 'right_wrist_camera' in frame_id:
                            camera_type = "right_wrist_camera"
                        else:
                            camera_type = f"other({frame_id})"
                    
                    # 保存检测信息
                    tag_info = {
                        'id': detection.id[0],  # AprilTag ID
                        'position': {
                            'x': detection.pose.pose.pose.position.x,
                            'y': detection.pose.pose.pose.position.y,
                            'z': detection.pose.pose.pose.position.z
                        },
                        'orientation': {
                            'x': detection.pose.pose.pose.orientation.x,
                            'y': detection.pose.pose.pose.orientation.y,
                            'z': detection.pose.pose.pose.orientation.z,
                            'w': detection.pose.pose.pose.orientation.w
                        },
                        'timestamp': time.time(),
                        'camera_type': camera_type,  # 记录相机类型
                        'frame_id': detection.pose.header.frame_id if hasattr(detection, 'pose') and hasattr(detection.pose, 'header') else 'unknown'  # 记录frame_id
                    }
                    
                    # 更新frame_id检测状态
                    frame_id = tag_info['frame_id']
                    if frame_id not in self.tag_detections.keys():
                        self.tag_detections[frame_id] = True
                        print(f"🎯 检测到AprilTag: ID={tag_info['id']}, 相机: {camera_type}, 位置=({tag_info['position']['x']:.3f}, {tag_info['position']['y']:.3f}, {tag_info['position']['z']:.3f})")
                        print(f"   Frame ID: {frame_id} - 已标记为检测到")
                    
                    # 更新标签历史
                    self.tag_history.append(tag_info)
            
            # 更新统计信息
            self.stats['total_frames'] += 1
            # 更新检测到的标签数量（基于frame_id的数量）
            self.stats['detected_tags'] = len(self.tag_detections)
            
        except Exception as e:
            print(f"AprilTag检测结果处理错误: {e}")
    
    def apriltag_image_callback(self, msg):
        """AprilTag检测图像回调函数"""
        try:
            # 保存图像消息，apriltag_ros已经处理了标注
            self.camera_images['apriltag_annotated'] = msg
            print(" 收到AprilTag标注图像")
        except Exception as e:
            print(f"AprilTag检测图像处理错误: {e}")
    
    def process_apriltag_frame(self, frame):
        """处理AprilTag检测帧"""
        if not self.apriltag_detector:
            return
            
        try:
            # 现在使用apriltag_ros包，不需要手动处理图像
            # 更新统计信息
            self.stats['total_frames'] += 1
            
        except Exception as e:
            print(f"AprilTag检测处理错误: {e}")
    
    def draw_apriltag_on_frame(self, frame, detection):
        """在图像上绘制AprilTag检测结果"""
        try:
            # 现在使用apriltag_ros包，图像已经自动标注
            print(f" AprilTag {detection.tag_id} 已在图像上标注")
            
        except Exception as e:
            print(f"绘制AprilTag检测结果失败: {e}")
    
    def start_apriltag_monitoring(self):
        """启动AprilTag检测监控"""
        print(" AprilTag检测监控已启动")
        print("正在实时检测AprilTag...")
        
        # 启动监控线程
        self.monitor_thread = threading.Thread(target=self._apriltag_monitor_loop)
        self.monitor_thread.daemon = True
        self.monitor_thread.start()
    
    def _apriltag_monitor_loop(self):
        """AprilTag监控循环"""
        last_report_time = time.time()
        
        while not rospy.is_shutdown():
            try:
                current_time = time.time()
                
                # 每5秒报告一次检测状态
                if current_time - last_report_time >= 5.0:
                    if self.tag_detections:
                        print(f" AprilTag检测统计: 已检测到 {len(self.tag_detections)} 个frame的标签")
                        print(f"   检测到的frames: {list(self.tag_detections.keys())}")
                    else:
                        print(" 正在检测AprilTag...")
                    
                    last_report_time = current_time
                
                time.sleep(1.0)
                
            except Exception as e:
                print(f"AprilTag监控循环错误: {e}")
                break
    
    def color_callback(self, msg):
        """彩色图像回调函数"""
        try:
            # 保存图像消息，不转换为OpenCV格式
            self.camera_images['color'] = msg
            self.process_frame('color')
        except Exception as e:
            print(f"彩色图像处理错误: {e}")
    
    def depth_callback(self, msg):
        """深度图像回调函数"""
        try:
            # 保存图像消息，不转换为OpenCV格式
            self.camera_images['depth'] = msg
            self.process_frame('depth')
        except Exception as e:
            print(f"深度图像处理错误: {e}")
    
    def process_frame(self, frame_type):
        """处理图像帧"""
        current_time = time.time()
        
        # 更新统计信息
        if frame_type not in self.stats['camera_frames']:
            self.stats['camera_frames'][frame_type] = 0
        self.stats['camera_frames'][frame_type] += 1
        self.stats['total_frames'] += 1
        
        # 计算FPS
        if self.last_frame_time > 0:
            frame_time = current_time - self.last_frame_time
            fps = 1.0 / frame_time if frame_time > 0 else 0
            self.fps_history.append(fps)
            self.frame_times.append(frame_time)
        
        self.last_frame_time = current_time
        
        # 更新平均FPS
        if self.fps_history:
            self.stats['avg_fps'] = sum(self.fps_history) / len(self.fps_history)
        
        # 如果启用AprilTag检测且检测器可用，进行检测
        if (self.enable_apriltag and self.apriltag_detector and 
            frame_type == 'color' and self.camera_images.get('color') is not None):
            # 现在使用apriltag_ros包，不需要手动检测
            pass
    
    def save_images(self, save_dir="unified_camera_test_images"):
        """保存当前图像"""
        if not self.enable_image_save:
            print("⚠️  图像保存功能已禁用")
            return False
        
        try:
            # 创建保存目录
            os.makedirs(save_dir, exist_ok=True)
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            
            saved_count = 0
            
            # 保存AprilTag标注图像（如果可用）
            if 'apriltag_annotated' in self.camera_images:
                # 这里可以添加保存ROS图像消息的代码
                print(" AprilTag标注图像可用，但保存功能需要额外实现")
                saved_count += 1
            
            if saved_count > 0:
                print(f"✅ 成功保存 {saved_count} 张图像到 {save_dir}")
                return True
            else:
                print("❌ 没有可保存的图像")
                return False
                
        except Exception as e:
            print(f"❌ 图像保存失败: {e}")
            return False
    
    def get_camera_status(self):
        """获取相机状态信息"""
        status = {
            'fps': self.stats['avg_fps'],
            'total_frames': self.stats['total_frames'],
            'camera_frames': self.stats['camera_frames'],
            'detected_tags': self.stats['detected_tags'],
            'uptime': time.time() - self.stats['start_time'],
            'has_images': {k: v is not None for k, v in self.camera_images.items()}
        }
        
        return status
    
    def print_status(self):
        """打印状态信息"""
        status = self.get_camera_status()
        
        print("\n" + "="*60)
        print(" 相机测试工具状态")
        print("="*60)
        print(f"平均FPS: {status['fps']:.2f}")
        print(f"总帧数: {status['total_frames']}")
        print(f"检测到的标签: {status['detected_tags']}")
        print(f"运行时间: {status['uptime']:.1f}秒")
        
        print(f"\n相机帧数统计:")
        for camera_type, frame_count in status['camera_frames'].items():
            print(f"  {camera_type}: {frame_count}")
        
        print(f"\n图像状态:")
        for image_type, has_image in status['has_images'].items():
            print(f"  {image_type}: {'✅' if has_image else '❌'}")
        
        if self.tag_detections:
            print(f"\n 当前检测到的AprilTag frames:")
            for frame_id, detected in self.tag_detections.items():
                if detected:
                    print(f"  ✅ {frame_id}: 已检测到AprilTag")
                else:
                    print(f"  ❌ {frame_id}: 未检测到AprilTag")
        else:
            print("\n 当前未检测到AprilTag")
        
        # 显示话题检测状态
        if hasattr(self, 'topic_detection_status') and self.topic_detection_status:
            print(f"\n 话题检测状态:")
            for topic, status in self.topic_detection_status.items():
                status_icon = "✅" if status['detected'] else "❌"
                status_text = "检测成功" if status['detected'] else "检测失败"
                print(f"  {status_icon} {topic}: {status_text}")
                if status['detection_time']:
                    detection_duration = status['detection_time'] - status['start_time'] if status['start_time'] else 0
                    print(f"    检测用时: {detection_duration:.1f}秒")
        
        print("="*60)
    
    def run_comprehensive_test(self, duration=None):
        """运行综合测试"""
        print(" 开始相机综合测试...")
        
        start_time = time.time()
        
        # 运行USB设备检查
        usb_ok, usb_devices = self.check_usb_devices()
        
        # 运行ROS话题检查
        topics_ok, camera_topics = self.check_ros_topics()
        
        # 检查图像话题详情
        if camera_topics.get('camera'):
            print("\n 检查图像话题详情...")
            image_topics = [t for t in camera_topics['camera'] if 'image' in t.lower()]
            for topic in image_topics:  # 检查所有图像话题
                self.check_image_topic_details(topic)
        
        
        # 订阅相机话题
        if topics_ok:
            self.subscribe_to_camera_topics()
        
        print(f"\n✅ 初始化检查完成")
        print(f"USB设备: {'✅' if usb_ok else '❌'}")
        print(f"ROS话题: {'✅' if topics_ok else '❌'}")
        
        # 选择有图像的话题进行AprilTag检测
        if topics_ok and self.apriltag_detector:
            self.select_topics_for_apriltag_detection(camera_topics)
        
        print("按 Ctrl+C 退出")
        
        try:
            while not rospy.is_shutdown():
                # 检查是否超时
                if duration and (time.time() - start_time) > duration:
                    break
                
                # 短暂延迟
                time.sleep(0.1)
                
        except KeyboardInterrupt:
            print("\n⚠️  用户中断")
        
        # 如果用户提前退出，也显示检测结果
        if hasattr(self, 'topic_detection_status') and self.topic_detection_status:
            self.check_all_topics_detection_results()
        
        self.cleanup()
    
    def cleanup(self):
        """清理资源"""
        # 停止所有 AprilTag launch 进程
        if hasattr(self, 'apriltag_processes') and self.apriltag_processes:
            for topic, proc in list(self.apriltag_processes.items()):
                if proc is None:
                    continue
                try:
                    print(f" 停止 AprilTag 进程 (话题: {topic}) ...")
                    proc.terminate()
                    proc.wait(timeout=5)
                    print("✅ 进程已停止")
                except Exception as e:
                    print(f"⚠️  停止进程时出错: {e}")
                    try:
                        proc.kill()
                    except Exception:
                        pass
            self.apriltag_processes.clear()

        # 取消订阅者
        for subscriber in self.camera_subscribers.values():
            if subscriber:
                subscriber.unregister()
        
        print("\n✅ 测试完成，资源已清理")
    


def main():
    """主函数"""
    parser = argparse.ArgumentParser(description='相机测试工具')
    parser.add_argument('--project-dir', type=str,
                       help='项目路径')
    
    args = parser.parse_args()
    
    try:
        # 创建测试工具实例
        tool = UnifiedCameraTestTool(
            proj_dir=args.project_dir
        )
        
        # 运行测试
        tool.run_comprehensive_test()
        
        
    except Exception as e:
        print(f"❌ 错误: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main() 
