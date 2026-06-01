#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
简单的 LED 演示脚本
快速演示 LED 的基本功能 - 纯硬件直连模式
"""

import sys
import time

try:
    import serial
except ImportError:
    import subprocess
    print("正在安装pyserial库...")
    subprocess.check_call([sys.executable, "-m", "pip", "install", "-i", "https://pypi.tuna.tsinghua.edu.cn/simple", "pyserial==3.5"])
    import serial

class LEDController:
    def __init__(self, port='/dev/ttyLED0', baudrate=115200):
        """
        初始化LED控制器
        :param port: 串口设备路径
        :param baudrate: 波特率
        """
        try:
            self.ser = serial.Serial(
                port=port,
                baudrate=baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=1
            )
            print(f"成功连接到设备: {port}")
        except serial.SerialException as e:
            print(f"无法连接到设备: {e}")
            print("请检查 UDEV 规则以及硬件设备是否正常！！")
            sys.exit(1)

    def calculate_checksum(self, data):
        """
        计算校验和
        :param data: 数据列表
        :return: 校验和
        """
        return (~sum(data)) & 0xFF

    def set_led_mode(self, mode, colors):
        """
        设置LED灯的模式和颜色
        :param mode: 模式 (0:常亮, 1:呼吸, 2:快闪, 3:律动)
        :param colors: 颜色列表，每个颜色为(R,G,B)元组
        """
        # 构建数据包
        packet = [0xFF, 0xFF, 0x00, 0x22, 0x02, 0x02, mode]
        
        # 添加颜色数据
        for r, g, b in colors:
            packet.extend([r, g, b])
        
        # 计算校验和
        checksum = self.calculate_checksum(packet[2:])
        packet.append(checksum)
        
        # 发送数据
        self.ser.write(bytes(packet))
        print(f"发送数据: {[hex(x) for x in packet]}")

    def close(self):
        """关闭串口连接"""
        self.ser.close()

    def deinit(self):
        self.set_led_mode(0x00, [(0, 0, 0)] * 10)
        self.close()

def run_demo():
    """运行 LED 演示"""
    try:
        print("🔥 Kuavo LED 演示开始...")
        print("=" * 40)
        
        # 初始化LED控制器
        led = LEDController("/dev/ttyLED0")
        
        # 演示序列
        demos = [
            {
                'name': '🔴 红色常亮',
                'mode': 0,
                'colors': [(255, 0, 0)] * 10,
                'duration': 2
            },
            {
                'name': '🟢 绿色呼吸',
                'mode': 1, 
                'colors': [(0, 255, 0)] * 10,
                'duration': 3
            },
            {
                'name': '🔵 蓝色快闪',
                'mode': 2,
                'colors': [(0, 0, 255)] * 10,
                'duration': 3
            },
            {
                'name': '🌈 彩虹律动',
                'mode': 3,
                'colors': [
                    (255, 0, 0), (255, 127, 0), (255, 255, 0), (0, 255, 0),
                    (0, 255, 255), (0, 0, 255), (75, 0, 130), (148, 0, 211),
                    (255, 0, 127), (255, 255, 255)
                ],
                'duration': 4
            },
            {
                'name': '💡 暖白色常亮',
                'mode': 0,
                'colors': [(255, 147, 41)] * 10,
                'duration': 2
            }
        ]
        
        # 执行演示
        for demo in demos:
            print(f"\n{demo['name']} ({demo['duration']}秒)")
            led.set_led_mode(demo['mode'], demo['colors'])
            time.sleep(demo['duration'])
        
        # 关闭LED
        print("\n⚫ 关闭所有LED")
        led.deinit()
        
        print("=" * 40)
        print("✅ LED 演示完成!")
        
    except KeyboardInterrupt:
        print("\n\n⚠️  演示被中断")
        if 'led' in locals():
            led.deinit()
    except Exception as e:
        print(f"\n❌ 演示过程中出错: {e}")
        if 'led' in locals():
            led.deinit()

if __name__ == '__main__':
    run_demo()
