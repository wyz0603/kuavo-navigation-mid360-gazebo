#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Kuavo LED 硬件测试脚本
专用于硬件直连模式测试
"""

import argparse
import time
import sys

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

class LEDTester:
    def __init__(self):
        """
        初始化LED测试器 - 仅支持硬件直连模式
        """
        self.led_controller = LEDController("/dev/ttyLED0")
    
    def get_preset_colors(self, preset_name):
        """获取预设颜色配置"""
        presets = {
            'rainbow': [
                (255, 0, 0),    # 红
                (255, 127, 0),  # 橙  
                (255, 255, 0),  # 黄
                (0, 255, 0),    # 绿
                (0, 255, 255),  # 青
                (0, 0, 255),    # 蓝
                (75, 0, 130),   # 靛
                (148, 0, 211),  # 紫
                (255, 0, 127),  # 粉
                (255, 255, 255) # 白
            ],
            'red': [(255, 0, 0)] * 10,
            'green': [(0, 255, 0)] * 10,
            'blue': [(0, 0, 255)] * 10,
            'white': [(255, 255, 255)] * 10,
            'off': [(0, 0, 0)] * 10,
            'warm': [(255, 147, 41)] * 10,  # 暖光
            'cool': [(173, 216, 230)] * 10,  # 冷光
            'gradient_red_blue': [
                (255, 0, 0), (230, 0, 25), (204, 0, 51), (179, 0, 76),
                (153, 0, 102), (128, 0, 127), (102, 0, 153), (76, 0, 179),
                (51, 0, 204), (0, 0, 255)
            ]
        }
        return presets.get(preset_name, presets['rainbow'])
    
    def test_mode(self, mode, colors, duration=3):
        """硬件模式测试"""
        if not self.led_controller:
            print("错误: LED控制器未初始化")
            return False
        
        try:
            mode_names = {0: "常亮", 1: "呼吸", 2: "快闪", 3: "律动"}
            print(f"LED测试 - {mode_names.get(mode, '未知')}模式, 持续时间: {duration}秒")
            self.led_controller.set_led_mode(mode, colors)
            time.sleep(duration)
            return True
        except Exception as e:
            print(f"LED测试失败: {e}")
            return False
    
    def run_full_test(self):
        """运行完整的LED测试序列"""
        print("开始 LED 完整测试...")
        print("=" * 50)
        
        test_sequences = [
            (0, 'red', "常亮模式 - 红色", 3),
            (1, 'green', "呼吸模式 - 绿色", 5),
            (2, 'blue', "快闪模式 - 蓝色", 5),
            (3, 'rainbow', "律动模式 - 彩虹", 5),
            (0, 'white', "常亮模式 - 白色", 3),
            (1, 'warm', "呼吸模式 - 暖光", 3),
            (2, 'cool', "快闪模式 - 冷光", 3),
            (0, 'gradient_red_blue', "常亮模式 - 红蓝渐变", 3)
        ]
        
        for mode, color_preset, description, duration in test_sequences:
            print(f"\n测试: {description}")
            colors = self.get_preset_colors(color_preset)
            success = self.test_mode(mode, colors, duration)
            if success:
                print(f"✓ {description} - 成功")
            else:
                print(f"✗ {description} - 失败")
        
        # 最后关闭所有LED
        print("\n关闭所有LED...")
        self.close_all_leds()
        print("=" * 50)
        print("LED 完整测试结束")
    
    def run_interactive_test(self):
        """运行交互式测试"""
        print("LED 交互式测试")
        print("=" * 30)
        
        while True:
            print("\n可用的测试选项:")
            print("1. 常亮模式测试")
            print("2. 呼吸模式测试") 
            print("3. 快闪模式测试")
            print("4. 律动模式测试")
            print("5. 彩虹模式测试")
            print("6. 自定义颜色测试")
            print("7. 关闭LED")
            print("0. 退出")
            
            try:
                choice = input("\n请选择测试项目 (0-7): ").strip()
                
                if choice == '0':
                    self.close_all_leds()
                    break
                elif choice == '1':
                    color = input("请输入RGB颜色 (例如: 255,0,0 表示红色): ").strip()
                    rgb = tuple(map(int, color.split(',')))
                    self.test_mode(0, [rgb] * 10, 3)
                elif choice == '2':
                    self.test_mode(1, self.get_preset_colors('green'), 5)
                elif choice == '3':
                    self.test_mode(2, self.get_preset_colors('blue'), 5)
                elif choice == '4':
                    self.test_mode(3, self.get_preset_colors('rainbow'), 5)
                elif choice == '5':
                    self.test_mode(0, self.get_preset_colors('rainbow'), 5)
                elif choice == '6':
                    print("自定义10个LED的颜色 (格式: R,G,B)")
                    colors = []
                    for i in range(10):
                        color_str = input(f"LED {i+1} 颜色: ").strip()
                        rgb = tuple(map(int, color_str.split(',')))
                        colors.append(rgb)
                    mode = int(input("选择模式 (0:常亮, 1:呼吸, 2:快闪, 3:律动): "))
                    self.test_mode(mode, colors, 5)
                elif choice == '7':
                    self.close_all_leds()
                else:
                    print("无效选择，请重试")
                    
            except KeyboardInterrupt:
                print("\n用户中断，退出测试")
                self.close_all_leds()
                break
            except Exception as e:
                print(f"测试出错: {e}")
    
    def close_all_leds(self):
        """关闭所有LED"""
        if self.led_controller:
            try:
                self.led_controller.deinit()
                print("LED已关闭")
            except Exception as e:
                print(f"关闭LED失败: {e}")
    
    def test_single_mode(self, mode, color_preset, duration):
        """测试单个模式"""
        colors = self.get_preset_colors(color_preset)
        mode_names = {0: "常亮", 1: "呼吸", 2: "快闪", 3: "律动"}
        print(f"\n测试 {mode_names.get(mode, '未知')} 模式 - {color_preset} 颜色")
        
        success = self.test_mode(mode, colors, duration)
        if success:
            print(f"✓ 测试成功")
        else:
            print(f"✗ 测试失败")
        
        self.close_all_leds()
        return success

def main():
    parser = argparse.ArgumentParser(description='Kuavo LED 硬件测试脚本')
    parser.add_argument('--test-type', choices=['full', 'interactive', 'single'], default='interactive',
                        help='测试类型: full(完整测试), interactive(交互式), single(单项测试)')
    parser.add_argument('--led-mode', type=int, choices=[0, 1, 2, 3],
                        help='LED模式 (仅用于单项测试): 0=常亮, 1=呼吸, 2=快闪, 3=律动')
    parser.add_argument('--color-preset', default='rainbow',
                        help='颜色预设 (仅用于单项测试): rainbow, red, green, blue, white, warm, cool')
    parser.add_argument('--duration', type=int, default=5,
                        help='测试持续时间(秒)')
    parser.add_argument('--list-presets', action='store_true',
                        help='列出所有可用的颜色预设')
    
    args = parser.parse_args()
    
    # 列出颜色预设
    if args.list_presets:
        print("可用的颜色预设:")
        presets = ['rainbow', 'red', 'green', 'blue', 'white', 'off', 'warm', 'cool', 'gradient_red_blue']
        for preset in presets:
            print(f"  - {preset}")
        return
    
    try:
        # 创建测试器
        tester = LEDTester()
        
        print("LED 硬件测试脚本启动")
        print("按 Ctrl+C 可随时退出并关闭LED")
        
        if args.test_type == 'full':
            tester.run_full_test()
        elif args.test_type == 'interactive':
            tester.run_interactive_test()
        elif args.test_type == 'single':
            if args.led_mode is None:
                print("错误: 单项测试需要指定 --led-mode 参数")
                return
            tester.test_single_mode(args.led_mode, args.color_preset, args.duration)
        
    except KeyboardInterrupt:
        print("\n用户中断测试")
    except Exception as e:
        print(f"测试过程中发生错误: {e}")
    finally:
        # 确保LED被关闭
        try:
            if 'tester' in locals():
                tester.close_all_leds()
        except:
            pass

if __name__ == '__main__':
    main()
