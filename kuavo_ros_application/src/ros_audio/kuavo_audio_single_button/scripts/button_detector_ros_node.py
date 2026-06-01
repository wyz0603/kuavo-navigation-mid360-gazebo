#!/usr/bin/env python3
"""
单按键检测器 ROS 节点
检测设备 4132:2107 的按键按下/释放事件，并发布到 ROS 话题
"""

import rospy
from std_msgs.msg import Bool
import os
import sys
import time
import select
import subprocess

def find_button_device():
    """查找按键设备，通过独有特征：4个HID接口组合 + vendor:product ID"""
    target_vendor = "4132"
    target_product = "2107"
    devices = []
    
    rospy.loginfo(f"正在搜索设备 {target_vendor}:{target_product}...")
    rospy.loginfo("独有特征：4个HID接口组合")
    
    # 通过USB设备查找，然后检查接口数量
    try:
        # 使用lsusb查找设备
        result = subprocess.run(['lsusb'], capture_output=True, text=True)
        usb_device_found = False
        
        for line in result.stdout.split('\n'):
            if f"{target_vendor}:{target_product}" in line:
                rospy.loginfo(f"找到USB设备: {line.strip()}")
                usb_device_found = True
                break
        
        if not usb_device_found:
            rospy.loginfo(f"USB设备 {target_vendor}:{target_product} 未连接")
            return devices
            
    except Exception as e:
        rospy.loginfo(f"检查USB设备时出错: {e}")
    
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
                                        rospy.loginfo(f"找到HID接口: {hidraw_path}")
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
        rospy.loginfo(f"✅ 验证通过：找到4个HID接口 - {device_hidraw_list}")
        devices.extend(device_hidraw_list)
    elif len(device_hidraw_list) > 0:
        rospy.logwarn(f"⚠️  找到 {len(device_hidraw_list)} 个HID接口，期望4个")
        rospy.logwarn("可能不是目标设备，但仍会尝试使用")
        devices.extend(device_hidraw_list)
    else:
        rospy.logwarn("未找到匹配的HID接口")
    
    # 如果hidraw没找到，尝试input设备
    if not devices:
        rospy.loginfo("尝试搜索input设备...")
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
                                        rospy.loginfo(f"找到input设备: {event_file}")
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
        rospy.logerr(f"❌ 未找到设备 {target_vendor}:{target_product}")
        rospy.logerr("请确认设备已连接并被系统识别")
        rospy.logerr("可以运行 'lsusb' 检查设备是否存在")
    
    return devices

def monitor_button():
    """监听按键事件并发布到ROS话题"""
    # 初始化ROS节点
    rospy.init_node('button_detector_ros_node', anonymous=True)
    
    # 创建发布者
    pub = rospy.Publisher('/kuavo/audio_single_button/event', Bool, queue_size=10)
    
    rospy.loginfo("按键检测器 ROS 节点已启动")
    
    # 查找所有可能的设备
    devices = find_button_device()
    
    if not devices:
        rospy.logerr("未找到按键设备!")
        return
    
    # 打开所有设备进行监听
    device_files = []
    for device_path in devices:
        try:
            device_file = open(device_path, 'rb')
            device_files.append((device_path, device_file))
            rospy.loginfo(f"✅ 成功打开: {device_path}")
        except Exception as e:
            rospy.logerr(f"❌ 无法打开 {device_path}: {e}")
    
    if not device_files:
        rospy.logerr("没有可用的设备文件!")
        return
    
    button_pressed = False
    press_time = 0
    target_device_found = False
    target_device_path = None
    
    rospy.loginfo("=" * 40)
    rospy.loginfo("单按键检测器 ROS 节点")
    rospy.loginfo(f"监听所有设备: {devices}")
    rospy.loginfo(f"发布话题: /kuavo/audio_single_button/event")
    rospy.loginfo("正在等待按键事件来识别目标设备...")
    rospy.loginfo("请按下/释放按键测试...")
    rospy.loginfo("按 Ctrl+C 停止")
    rospy.loginfo("=" * 40)
    
    try:
        while not rospy.is_shutdown():
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
                                    rospy.loginfo(f"🔴 [{timestamp}] 按键按下")
                                    
                                    # 发布按键按下事件
                                    msg = Bool()
                                    msg.data = True
                                    pub.publish(msg)
                                    
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
                                    
                                    rospy.loginfo(f"🔵 [{timestamp}] 按键释放 - 持续: {hold_duration:.3f}秒 ({key_type})")
                                    
                                    # 发布按键释放事件
                                    msg = Bool()
                                    msg.data = False
                                    pub.publish(msg)
                                    
                        except Exception as e:
                            rospy.logerr(f"读取目标设备时出错: {e}")
                else:
                    rospy.logerr("目标设备文件丢失")
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
                                rospy.loginfo(f"🎯 找到按键设备: {device_path}")
                                rospy.loginfo(f"   报告ID: 0x29")
                                rospy.loginfo(f"   按键状态: 0x{data[1]:02x}")
                                rospy.loginfo("现在只监听目标设备...")
                                
                                # 关闭其他设备文件
                                for path, file in device_files:
                                    if path != target_device_path:
                                        try:
                                            file.close()
                                            rospy.loginfo(f"关闭非目标设备: {path}")
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
                                    rospy.loginfo(f"🔴 [{timestamp}] 按键按下")
                                    
                                    msg = Bool()
                                    msg.data = True
                                    pub.publish(msg)
                                
                                break
                            else:
                                # 其他数据，显示调试信息
                                rospy.logdebug(f"设备 {device_path} 发送数据: {data[:10].hex()}...")
                                
                    except Exception as e:
                        rospy.logerr(f"读取 {device_path} 时出错: {e}")
                    
    except PermissionError:
        rospy.logerr("权限不足，请先切换到root用户(sudo su)后再运行")
    except KeyboardInterrupt:
        rospy.loginfo("\n按键监听已停止")
    except Exception as e:
        rospy.logerr(f"监听失败: {e}")
    finally:
        # 关闭所有设备文件
        for device_path, device_file in device_files:
            try:
                device_file.close()
                rospy.loginfo(f"已关闭: {device_path}")
            except Exception:
                pass

def main():
    if os.geteuid() != 0:
        print("需要root权限来访问设备文件")
        print("请先切换到root用户: sudo su")
        print("然后运行: rosrun kuavo_audio_single_button button_detector_ros_node.py")
        sys.exit(1)
    
    try:
        monitor_button()
    except rospy.ROSInterruptException:
        pass

if __name__ == "__main__":
    main()