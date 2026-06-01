#!/usr/bin/env python3
"""
单按键检测器 - 简化版
检测设备 4132:2107 的按键按下/释放事件
"""

import os
import sys
import time
import select

def find_button_device():
    """查找按键设备，通过独有特征：4个HID接口组合 + vendor:product ID"""
    target_vendor = "4132"
    target_product = "2107"
    devices = []
    
    print(f"正在搜索设备 {target_vendor}:{target_product}...")
    print("独有特征：4个HID接口组合")
    
    # 通过USB设备查找，然后检查接口数量
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
    
    # 查找该设备对应的所有HID接口
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
    
    # 如果hidraw没找到，尝试input设备
    if not devices:
        print("尝试搜索input设备...")
        from pathlib import Path
        
        for event_file in Path("/dev/input").glob("event*"):
            try:
                device_info_path = f"/sys/class/input/{event_file.name}/device"
                if os.path.exists(device_info_path):
                    current = device_info_path
                    for _ in range(5):
                        uevent_file = os.path.join(current, "uevent")
                        if os.path.exists(uevent_file):
                            try:
                                with open(uevent_file, 'r') as f:
                                    content = f.read()
                                    # 检查多种格式
                                    if (f"PRODUCT={target_vendor}/{target_product}" in content or
                                        f"HID_ID=0003:0000{target_vendor.upper()}:0000{target_product.upper()}" in content or
                                        f"HID_NAME=HID {target_vendor}:{target_product}" in content):
                                        devices.append(str(event_file))
                                        print(f"找到input设备: {event_file}")
                                        break
                            except:
                                pass
                        parent = os.path.dirname(current)
                        if parent == current:
                            break
                        current = parent
            except Exception as e:
                continue
    
    if not devices:
        print(f"❌ 未找到设备 {target_vendor}:{target_product}")
        print("请确认设备已连接并被系统识别")
        print("可以运行 'lsusb' 检查设备是否存在")
    
    return devices

def monitor_button():
    """监听按键事件"""
    devices = find_button_device()
    
    if not devices:
        print("未找到按键设备!")
        return
    
    # 打开所有设备进行监听
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
    
    button_pressed = False
    press_time = 0
    target_device_found = False
    target_device_path = None
    
    print("=" * 40)
    print("单按键检测器")
    print(f"监听所有设备: {devices}")
    print("正在等待按键事件来识别目标设备...")
    print("请按下/释放按键测试...")
    print("按 Ctrl+C 停止")
    print("=" * 40)
    
    try:
        while True:
            if target_device_found:
                # 已找到目标设备，只监听目标设备
                target_file = None
                for path, file in device_files:
                    if path == target_device_path:
                        target_file = file
                        break
                
                if target_file:
                    readable, _, _ = select.select([target_file], [], [], 1.0)
                    
                    if readable:
                        try:
                            # 读取数据
                            data = target_file.read(64)
                            if data and len(data) >= 2 and data[0] == 0x29:
                                current_time = time.time()
                                
                                if data[1] == 0x01 and not button_pressed:
                                    # 按键按下
                                    button_pressed = True
                                    press_time = current_time
                                    timestamp = time.strftime("%H:%M:%S", time.localtime(current_time))
                                    print(f"\n🔴 [{timestamp}] 按键按下")
                                    
                                elif data[1] == 0x00 and button_pressed:
                                    # 按键释放
                                    button_pressed = False
                                    hold_duration = current_time - press_time
                                    timestamp = time.strftime("%H:%M:%S", time.localtime(current_time))
                                    
                                    # 判断按键类型
                                    if hold_duration < 0.2:
                                        key_type = "快速点击"
                                    elif hold_duration < 1.0:
                                        key_type = "正常按键"
                                    else:
                                        key_type = "长按"
                                    
                                    print(f"🔵 [{timestamp}] 按键释放 - 持续: {hold_duration:.3f}秒 ({key_type})")
                                    
                        except Exception as e:
                            print(f"读取目标设备时出错: {e}")
                else:
                    print("目标设备文件丢失")
                    break
            else:
                # 还未找到目标设备，监听所有设备
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
                            # 检查是否是预期的0x29报告
                            if len(data) >= 2 and data[0] == 0x29:
                                # 找到目标设备！
                                target_device_found = True
                                target_device_path = device_path
                                print(f"🎯 找到按键设备: {device_path}")
                                print("   报告ID: 0x29")
                                print(f"   按键状态: 0x{data[1]:02x}")
                                print("现在只监听目标设备...")
                                
                                # 关闭其他设备文件
                                for path, file in device_files:
                                    if path != target_device_path:
                                        try:
                                            file.close()
                                            print(f"关闭非目标设备: {path}")
                                        except Exception:
                                            pass
                                
                                # 只保留目标设备
                                device_files = [(target_device_path, device_file)]
                                
                                # 处理当前的按键事件
                                current_time = time.time()
                                if data[1] == 0x01 and not button_pressed:
                                    button_pressed = True
                                    press_time = current_time
                                    timestamp = time.strftime("%H:%M:%S", time.localtime(current_time))
                                    print(f"\n🔴 [{timestamp}] 按键按下")
                                
                                break
                            else:
                                # 其他数据，显示调试信息
                                print(f"设备 {device_path} 发送数据: {data[:10].hex()}...")
                                
                    except Exception as e:
                        print(f"读取 {device_path} 时出错: {e}")
                    
    except PermissionError:
        print(f"权限不足，请使用: sudo python3 {sys.argv[0]}")
    except KeyboardInterrupt:
        print(f"\n\n按键监听已停止")
    except Exception as e:
        print(f"监听失败: {e}")
    finally:
        # 关闭所有设备文件
        for device_path, device_file in device_files:
            try:
                device_file.close()
                print(f"已关闭: {device_path}")
            except Exception:
                pass

def main():
    if len(sys.argv) > 1 and sys.argv[1] == "--help":
        print("单按键检测器")
        print("检测设备 4132:2107 的按键按下/释放事件")
        print(f"\n使用方法: sudo python3 {sys.argv[0]}")
        return
    
    if os.geteuid() != 0:
        print("需要root权限来访问设备文件")
        print(f"请使用: sudo python3 {sys.argv[0]}")
        sys.exit(1)
    
    monitor_button()

if __name__ == "__main__":
    main()
