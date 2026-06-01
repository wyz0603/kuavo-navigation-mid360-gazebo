#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
雷达检测工具
用于检测雷达点云频率和点云总数，判断雷达工作状态
"""

import rospy
import time
import numpy as np
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header
import argparse
import sys
import os

class LidarTestTool:
    def __init__(self, duration=30, min_frequency=5.0, min_point_count=1000):
        """
        初始化雷达检测工具
        
        Args:
            duration (int): 检测持续时间（秒）
            min_frequency (float): 最小点云频率阈值（Hz）
            min_point_count (int): 最小点云总数阈值
        """
        self.duration = duration
        self.min_frequency = min_frequency
        self.min_point_count = min_point_count
        
        # 统计数据
        self.point_cloud_count = 0
        self.total_points = 0
        self.timestamps = []
        self.point_counts = []
        
        # 状态标志
        self.is_running = False
        self.start_time = None
        self.current_stage = "frequency"  # "frequency" 或 "point_cloud"
        
        # 初始化ROS节点
        rospy.init_node('lidar_test_tool', anonymous=True)
        
        # 订阅点云话题
        self.point_cloud_sub = rospy.Subscriber(
            '/lidar',  # 默认话题，可通过参数修改
            PointCloud2,
            self.point_cloud_callback,
            queue_size=10
        )
        
        print(f"🚀 雷达检测工具已启动")
        print(f"📊 检测参数:")
        print(f"   - 检测时长: {duration} 秒")
        print(f"   - 最小频率阈值: {min_frequency} Hz")
        print(f"   - 最小点云总数阈值: {min_point_count}")
        print(f"   - 订阅话题: /lidar")
        print(f"⏳ 等待点云数据...")
        print(f"💡 提示:")
        print(f"   - 确保雷达已连接并正常工作")
        print(f"   - 确保ROS环境已正确加载")
        print(f"   - 使用 'rostopic list' 查看可用话题")
        print(f"   - 按 Ctrl+C 可以中断检测")
        print(f"   - 先检测频率，再检测点云数据")
        
    def point_cloud_callback(self, msg):
        """点云数据回调函数"""
        if not self.is_running:
            return
            
        current_time = time.time()
        
        if self.current_stage == "frequency":
            # 第一阶段：频率检测 - 只记录时间戳，不处理点云数据
            self.point_cloud_count += 1
            self.timestamps.append(current_time)
            
            # 实时显示频率状态
            if self.point_cloud_count % 10 == 0:  # 每10帧显示一次
                elapsed_time = current_time - self.start_time
                current_frequency = self.point_cloud_count / elapsed_time if elapsed_time > 0 else 0
                
                print(f"📈 频率检测 - 帧数: {self.point_cloud_count}, "
                      f"频率: {current_frequency:.2f} Hz, "
                      f"剩余时间: {self.duration - elapsed_time:.1f}s")
        else:
            # 点云数据检测 - 处理点云数据和频率
            point_count = len(list(pc2.read_points(msg)))
            
            # 记录数据
            self.point_cloud_count += 1
            self.total_points += point_count
            self.timestamps.append(current_time)
            self.point_counts.append(point_count)
            
            # 实时显示状态
            if self.point_cloud_count % 10 == 0:  # 每10帧显示一次
                elapsed_time = current_time - self.start_time
                current_frequency = self.point_cloud_count / elapsed_time if elapsed_time > 0 else 0
                avg_point_count = self.total_points / self.point_cloud_count if self.point_cloud_count > 0 else 0
                
                print(f"📈 点云检测 - 帧数: {self.point_cloud_count}, "
                      f"频率: {current_frequency:.2f} Hz, "
                      f"平均点数: {avg_point_count:.0f}, "
                      f"剩余时间: {self.duration - elapsed_time:.1f}s")
    
    def start_detection(self):
        """开始检测"""
        print(f"🔍 开始检测...")
        print(f"⏱️  检测将持续 {self.duration} 秒")
        print(f"📊 先检测频率，再检测点云数据")
        print(f"💡 注意：频率检测时不分析点云内容")
        self.is_running = True
        self.start_time = time.time()
        
        # 频率检测阶段
        print(f"\n🔄 开始频率检测")
        self.current_stage = "frequency"
        self.point_cloud_count = 0
        self.timestamps = []
        
        frequency_duration = self.duration // 2
        rate = rospy.Rate(1)  # 1Hz检查频率
        while not rospy.is_shutdown():
            elapsed_time = time.time() - self.start_time
            if elapsed_time >= frequency_duration:
                break
            rate.sleep()
        
        # 频率检测完成，显示结果
        frequency_ok = self.show_frequency_report()
        
        # 如果频率检测失败，直接退出
        if not frequency_ok:
            print("\n❌ 频率检测失败，检测终止！")
            print("💡 请检查雷达连接和驱动状态后重试")
            return
        
        # 点云数据检测阶段
        print(f"\n🔄 开始点云数据检测")
        self.current_stage = "point_cloud"
        self.point_cloud_count = 0
        self.total_points = 0
        self.timestamps = []
        self.point_counts = []
        
        # 重置开始时间
        self.start_time = time.time()
        
        while not rospy.is_shutdown():
            elapsed_time = time.time() - self.start_time
            if elapsed_time >= frequency_duration:
                break
            rate.sleep()
        
        # 点云检测完成，显示完整结果
        self.stop_detection()
    
    def show_frequency_report(self):
        """显示频率检测报告"""
        if not self.timestamps:
            print("❌ 频率检测阶段未接收到任何数据！")
            return False
        
        # 计算频率统计数据
        elapsed_time = self.timestamps[-1] - self.start_time
        frequency = self.point_cloud_count / elapsed_time if elapsed_time > 0 else 0
        
        # 计算频率稳定性
        if len(self.timestamps) > 1:
            intervals = np.diff(self.timestamps)
            frequency_stability = np.std(intervals)
            min_interval = np.min(intervals)
            max_interval = np.max(intervals)
        else:
            frequency_stability = 0
            min_interval = 0
            max_interval = 0
        
        print("\n" + "="*60)
        print("📊 频率检测报告")
        print("="*60)
        print(f"⏱️  检测时长: {elapsed_time:.2f} 秒")
        print(f"📈 接收帧数: {self.point_cloud_count}")
        print(f"⚡ 平均频率: {frequency:.2f} Hz")
        print(f"📏 频率稳定性: {frequency_stability:.4f} 秒")
        print(f"⏱️  最小间隔: {min_interval:.4f} 秒")
        print(f"⏱️  最大间隔: {max_interval:.4f} 秒")
        
        # 判断频率结果
        print("\n" + "="*60)
        print("🎯 频率检测结果")
        print("="*60)
        
        frequency_ok = frequency >= self.min_frequency
        
        if frequency_ok:
            print("✅ 雷达频率正常！")
            print(f"   - 频率 ({frequency:.2f} Hz) >= 阈值 ({self.min_frequency} Hz)")
            print("\n💡 提示:")
            print("   - 雷达数据发布频率符合要求")
            print("   - 继续进行点云数据检测")
        else:
            print("❌ 雷达频率异常！")
            print(f"   - 频率 ({frequency:.2f} Hz) < 阈值 ({self.min_frequency} Hz)")
            print("\n🔧 故障排除建议:")
            print("   - 检查雷达配置参数，确认扫描频率设置")
            print("   - 检查网络带宽和系统资源使用情况")
            print("   - 重启雷达驱动或重新插拔USB连接")
            print("\n📋 调试命令:")
            print("   - rostopic list | grep lidar")
            print("   - rostopic echo /lidar -n 1")
            print("   - rosnode list | grep livox")
            print("\n⚠️  频率异常，但将继续进行点云检测")
        
        # 返回频率检测是否成功
        return frequency_ok
    
    def stop_detection(self):
        """停止检测并生成报告"""
        self.is_running = False
        
        if not self.timestamps:
            print("❌ 未接收到任何点云数据！")
            return
        
        # 计算统计数据
        elapsed_time = self.timestamps[-1] - self.start_time
        frequency = self.point_cloud_count / elapsed_time if elapsed_time > 0 else 0
        avg_point_count = self.total_points / self.point_cloud_count if self.point_cloud_count > 0 else 0
        
        # 计算频率稳定性
        if len(self.timestamps) > 1:
            intervals = np.diff(self.timestamps)
            frequency_stability = np.std(intervals)
            min_interval = np.min(intervals)
            max_interval = np.max(intervals)
        else:
            frequency_stability = 0
            min_interval = 0
            max_interval = 0
        
        print("\n" + "="*60)
        print("📊 雷达点云检测报告")
        print("="*60)
        print(f"⏱️  检测时长: {elapsed_time:.2f} 秒")
        print(f"📈 接收帧数: {self.point_cloud_count}")
        print(f"🔢 总点数: {self.total_points}")
        print(f"📊 平均每帧点数: {avg_point_count:.0f}")
        print(f"⚡ 平均频率: {frequency:.2f} Hz")
        print(f"📏 频率稳定性: {frequency_stability:.4f} 秒")
        print(f"⏱️  最小间隔: {min_interval:.4f} 秒")
        print(f"⏱️  最大间隔: {max_interval:.4f} 秒")
        
        # 判断结果
        print("\n" + "="*60)
        print("🎯 检测结果")
        print("="*60)
        
        # 点云检测结果
        frequency_ok = frequency >= self.min_frequency
        point_count_ok = avg_point_count >= self.min_point_count
        
        if frequency_ok and point_count_ok:
            print("✅ 雷达工作正常！")
            print(f"   - 频率 ({frequency:.2f} Hz) >= 阈值 ({self.min_frequency} Hz)")
            print(f"   - 平均点数 ({avg_point_count:.0f}) >= 阈值 ({self.min_point_count})")
            print("\n💡 提示:")
            print("   - 雷达性能良好，可以正常使用")
            print("   - 建议定期进行检测以监控性能变化")
        else:
            print("❌ 雷达工作异常！")
            if not frequency_ok:
                print(f"   - 频率 ({frequency:.2f} Hz) < 阈值 ({self.min_frequency} Hz)")
            if not point_count_ok:
                print(f"   - 平均点数 ({avg_point_count:.0f}) < 阈值 ({self.min_point_count})")
            
            print("\n🔧 故障排除建议:")
            if not frequency_ok:
                print("   - 检查雷达配置参数，确认扫描频率设置")
                print("   - 检查网络带宽和系统资源使用情况")
                print("   - 重启雷达驱动或重新插拔USB连接")
            if not point_count_ok:
                print("   - 检查雷达扫描范围和环境遮挡情况")
                print("   - 确认雷达安装高度和角度是否合适")
                print("   - 检查雷达镜头是否有污垢或损坏")
            
            print("\n📋 调试命令:")
            print("   - rostopic list | grep lidar")
            print("   - rostopic echo /lidar -n 1")
            print("   - rosnode list | grep livox")
            print("   - rosnode info /livox_lidar_publisher")
    

def main():
    """主函数"""
    parser = argparse.ArgumentParser(description='雷达检测工具')
    parser.add_argument('--duration', type=int, default=30, 
                       help='检测持续时间（秒），默认30秒')
    parser.add_argument('--min-frequency', type=float, default=5.0,
                       help='最小点云频率阈值（Hz），默认5.0')
    parser.add_argument('--min-point-count', type=int, default=1000,
                       help='最小点云总数阈值，默认1000')
    parser.add_argument('--topic', type=str, default='/lidar',
                       help='点云话题名称，默认/lidar')

    
    args = parser.parse_args()
    
    try:
        # 创建检测工具实例
        tool = LidarTestTool(
            duration=args.duration,
            min_frequency=args.min_frequency,
            min_point_count=args.min_point_count
        )
        
        # 修改订阅的话题
        if args.topic != '/lidar':
            tool.point_cloud_sub.unregister()
            tool.point_cloud_sub = rospy.Subscriber(
                args.topic,
                PointCloud2,
                tool.point_cloud_callback,
                queue_size=10
            )
            print(f"📡 已切换到话题: {args.topic}")
        
        # 开始检测
        tool.start_detection()
        
    except KeyboardInterrupt:
        print("\n⚠️  用户中断检测")
        if 'tool' in locals():
            tool.stop_detection()
    except Exception as e:
        print(f"❌ 检测过程中出现错误: {e}")
        rospy.logerr(f"检测错误: {e}")

if __name__ == '__main__':
    main() 