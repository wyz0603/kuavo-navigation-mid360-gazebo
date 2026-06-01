#!/usr/bin/env python3
"""
设备查找脚本
帮助用户找到正确的hidraw设备并生成配置
"""

import os
import sys
import time
import select
import json

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

def monitor_and_find_button_device():
    """监听所有设备，找到发送按键事件的设备"""
    devices = find_button_device()
    
    if not devices:
        print("未找到按键设备!")
        return None
    
    print("=" * 60)
    print("设备查找模式")
    print(f"监听所有设备: {devices}")
    print("请按下按键，系统将自动识别发送按键事件的设备")
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
        return None
    
    button_device = None
    
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
                    if data and len(data) >= 2 and data[0] == 0x29:
                        if data[1] == 0x01:  # 按键按下
                            button_device = device_path
                            print(f"\n🎯 找到按键设备: {device_path}")
                            print(f"   报告ID: 0x{data[0]:02x}")
                            print(f"   按键状态: 0x{data[1]:02x}")
                            return button_device
                            
                except Exception as e:
                    print(f"读取 {device_path} 时出错: {e}")
                    
    except KeyboardInterrupt:
        print(f"\n\n设备查找已停止")
    except Exception as e:
        print(f"监听失败: {e}")
    finally:
        # 关闭所有设备文件
        for device_path, device_file in device_files:
            try:
                device_file.close()
            except:
                pass
    
    return button_device

def create_config(button_device):
    """创建配置文件"""
    if not button_device:
        print("未找到按键设备，无法创建配置")
        return
    
    config = {
        "button_device": button_device,
        "vendor_id": "4132",
        "product_id": "2107",
        "created_at": time.strftime("%Y-%m-%d %H:%M:%S"),
        "description": "单按键设备配置"
    }
    
    config_file = os.path.join(os.path.dirname(__file__), "button_config.json")
    
    try:
        with open(config_file, 'w') as f:
            json.dump(config, f, indent=2, ensure_ascii=False)
        print(f"✅ 配置文件已保存: {config_file}")
        print(f"   按键设备: {button_device}")
    except Exception as e:
        print(f"❌ 保存配置文件失败: {e}")

def create_udev_rule():
    """创建udev规则"""
    udev_rule = '''# 单按键设备 udev 规则
# 为设备 4132:2107 创建固定的符号链接

SUBSYSTEM=="hidraw", ATTRS{idVendor}=="4132", ATTRS{idProduct}=="2107", SYMLINK+="kuavo_button"

# 可选：设置权限
SUBSYSTEM=="hidraw", ATTRS{idVendor}=="4132", ATTRS{idProduct}=="2107", MODE="0666"
'''
    
    rule_file = "/etc/udev/rules.d/99-kuavo-button.rules"
    
    print("=" * 60)
    print("udev 规则创建")
    print("=" * 60)
    print("udev 规则内容:")
    print(udev_rule)
    print(f"规则文件路径: {rule_file}")
    print("\n要应用此规则，请执行以下命令:")
    print(f"sudo cp /dev/null {rule_file}")
    print(f"echo '{udev_rule}' | sudo tee {rule_file}")
    print("sudo udevadm control --reload-rules")
    print("sudo udevadm trigger")
    print("\n应用后，设备将可通过 /dev/kuavo_button 访问")

def apply_udev_rule():
    """直接应用udev规则"""
    udev_rule = '''# 单按键设备 udev 规则
# 为设备 4132:2107 创建固定的符号链接

SUBSYSTEM=="hidraw", ATTRS{idVendor}=="4132", ATTRS{idProduct}=="2107", SYMLINK+="kuavo_button"

# 可选：设置权限
SUBSYSTEM=="hidraw", ATTRS{idVendor}=="4132", ATTRS{idProduct}=="2107", MODE="0666"
'''
    
    rule_file = "/etc/udev/rules.d/99-kuavo-button.rules"
    
    print("正在应用 udev 规则...")
    
    try:
        # 创建规则文件
        with open(rule_file, 'w') as f:
            f.write(udev_rule)
        print(f"✅ 规则文件已创建: {rule_file}")
        
        # 重新加载规则
        import subprocess
        result = subprocess.run(['udevadm', 'control', '--reload-rules'], capture_output=True, text=True)
        if result.returncode == 0:
            print("✅ udev 规则已重新加载")
        else:
            print(f"⚠️  重新加载规则时出错: {result.stderr}")
        
        # 触发规则
        result = subprocess.run(['udevadm', 'trigger'], capture_output=True, text=True)
        if result.returncode == 0:
            print("✅ udev 规则已触发")
        else:
            print(f"⚠️  触发规则时出错: {result.stderr}")
        
  
        print("请尝试重新插拔 USB 设备")
        
    except Exception as e:
        print(f"❌ 应用 udev 规则失败: {e}")
        return False
    
    return True

def main():
    if len(sys.argv) > 1:
        if sys.argv[1] == "--udev":
            create_udev_rule()
            return
        elif sys.argv[1] == "--help":
            print("设备查找脚本")
            print("使用方法:")
            print(f"  python3 {sys.argv[0]}              # 查找设备并创建配置")
            print(f"  python3 {sys.argv[0]} --udev       # 创建udev规则")
            print(f"  python3 {sys.argv[0]} --help       # 显示帮助")
            return
    
    if os.geteuid() != 0:
        print("需要root权限来访问设备文件")
        print(f"请使用: sudo python3 {sys.argv[0]}")
        sys.exit(1)
    
    print("单按键设备查找工具")
    print("=" * 60)
    
    # 查找按键设备
    button_device = monitor_and_find_button_device()
    
    if button_device:
        # 创建配置文件
        create_config(button_device)
        
        # 询问是否创建udev规则
        print("\n是否要创建并应用udev规则？(y/n): ", end="")
        try:
            response = input().strip().lower()
            if response in ['y', 'yes', '是']:
                print("\n选择操作:")
                print("1. 仅显示规则内容")
                print("2. 直接应用规则")
                print("请选择 (1/2): ", end="")
                choice = input().strip()
                if choice == "1":
                    create_udev_rule()
                elif choice == "2":
                    apply_udev_rule()
                else:
                    print("无效选择，仅显示规则内容")
                    create_udev_rule()
        except KeyboardInterrupt:
            print("\n已取消")
    else:
        print("未找到按键设备，请检查设备连接")

if __name__ == "__main__":
    main()
