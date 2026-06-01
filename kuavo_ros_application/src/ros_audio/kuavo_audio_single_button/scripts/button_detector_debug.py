#!/usr/bin/env python3
"""
单按键检测器 - 调试版
检测设备 4132:2107 的按键按下/释放事件，显示所有原始数据
"""

import os
import sys
import time
import select

def find_button_device():
    """通过独有特征查找按键设备：4个HID接口组合 + vendor:product ID"""
    target_vendor = "4132"
    target_product = "2107"
    devices = []
    
    print(f"正在搜索设备 {target_vendor}:{target_product}...")
    print("独有特征：4个HID接口组合")
    
    # 方法1: 通过USB设备查找，然后检查接口数量
    import subprocess
    try:
        # 使用lsusb查找设备
        result = subprocess.run(['lsusb'], capture_output=True, text=True)
        usb_device_found = False
        
        for line in result.stdout.split('\n'):
            if f"{target_vendor}:{target_product}" in line:
                print(f"找到USB设备: {line.strip()}")
                usb_device_found = True
                break
        
        if not usb_device_found:
            print(f"USB设备 {target_vendor}:{target_product} 未连接")
            return devices
            
    except Exception as e:
        print(f"检查USB设备时出错: {e}")
    
    # 方法2: 查找该设备对应的所有HID接口
    device_hidraw_list = []
    
    for i in range(10):  # 检查hidraw0-9
        hidraw_path = f"/dev/hidraw{i}"
        if os.path.exists(hidraw_path):
            try:
                # 检查设备的USB信息
                sys_path = f"/sys/class/hidraw/hidraw{i}/device"
                if os.path.exists(sys_path):
                    # 向上查找USB设备信息
                    current = sys_path
                    for _ in range(5):  # 最多向上查找5级
                        uevent_file = os.path.join(current, "uevent")
                        if os.path.exists(uevent_file):
                            try:
                                with open(uevent_file, 'r') as f:
                                    content = f.read()
                                    # 检查多种格式
                                    if (f"PRODUCT={target_vendor}/{target_product}" in content or
                                        f"HID_ID=0003:0000{target_vendor.upper()}:0000{target_product.upper()}" in content or
                                        f"HID_NAME=HID {target_vendor}:{target_product}" in content):
                                        device_hidraw_list.append(hidraw_path)
                                        print(f"找到HID接口: {hidraw_path}")
                                        break
                            except:
                                pass
                        # 向上一级目录
                        parent = os.path.dirname(current)
                        if parent == current:  # 已到根目录
                            break
                        current = parent
            except Exception as e:
                continue
    
    # 验证独有特征：应该有4个HID接口
    if len(device_hidraw_list) == 4:
        print(f"✅ 验证通过：找到4个HID接口 - {device_hidraw_list}")
        devices.extend(device_hidraw_list)
    elif len(device_hidraw_list) > 0:
        print(f"⚠️  找到 {len(device_hidraw_list)} 个HID接口，期望4个")
        print("可能不是目标设备，但仍会尝试使用")
        devices.extend(device_hidraw_list)
    else:
        print("未找到匹配的HID接口")
    
    return devices

def monitor_all_devices():
    """监听所有找到的设备，显示原始数据"""
    devices = find_button_device()
    
    if not devices:
        print("未找到按键设备!")
        return
    
    print("=" * 60)
    print("调试版单按键检测器")
    print(f"监听所有设备: {devices}")
    print("将显示所有接收到的原始数据")
    print("请按下/释放按键测试...")
    print("按 Ctrl+C 停止")
    print("=" * 60)
    
    # 打开所有设备
    device_files = []
    for device_path in devices:
        try:
            device_file = open(device_path, 'rb')
            device_files.append((device_path, device_file))
            print(f"✅ 成功打开: {device_path}")
        except Exception as e:
            print(f"❌ 无法打开 {device_path}: {e}")
    
    if not device_files:
        print("没有可用的设备文件!")
        return
    
    try:
        while True:
            # 使用select监听所有设备
            readable, _, _ = select.select([f[1] for f in device_files], [], [], 1.0)
            
            for device_file in readable:
                # 找到对应的设备路径
                device_path = None
                for path, file in device_files:
                    if file == device_file:
                        device_path = path
                        break
                
                try:
                    # 读取数据
                    data = device_file.read(64)
                    if data:
                        current_time = time.time()
                        timestamp = time.strftime("%H:%M:%S.%f", time.localtime(current_time))[:-3]
                        
                        # 显示原始数据
                        hex_data = ' '.join([f'{b:02x}' for b in data])
                        print(f"\n[{timestamp}] {device_path}: {hex_data}")
                        
                        # 分析数据
                        if len(data) >= 2:
                            report_id = data[0]
                            print(f"  报告ID: 0x{report_id:02x}")
                            
                            if report_id == 0x29 and len(data) >= 2:
                                button_state = data[1]
                                print(f"  按键状态: 0x{button_state:02x} ({button_state})")
                                if button_state == 0x01:
                                    print("  🔴 按键按下!")
                                elif button_state == 0x00:
                                    print("  🔵 按键释放!")
                            
                            # 显示所有字节的含义
                            print(f"  数据解析: {[f'0x{b:02x}' for b in data[:8]]}")
                        
                except Exception as e:
                    print(f"读取 {device_path} 时出错: {e}")
                    
    except KeyboardInterrupt:
        print(f"\n\n调试监听已停止")
    except Exception as e:
        print(f"监听失败: {e}")
    finally:
        # 关闭所有设备文件
        for device_path, device_file in device_files:
            try:
                device_file.close()
                print(f"已关闭: {device_path}")
            except:
                pass

def main():
    if len(sys.argv) > 1 and sys.argv[1] == "--help":
        print("调试版单按键检测器")
        print("检测设备 4132:2107 的按键按下/释放事件，显示所有原始数据")
        print(f"\n使用方法: sudo python3 {sys.argv[0]}")
        return
    
    if os.geteuid() != 0:
        print("需要root权限来访问设备文件")
        print(f"请使用: sudo python3 {sys.argv[0]}")
        sys.exit(1)
    
    monitor_all_devices()

if __name__ == "__main__":
    main()
