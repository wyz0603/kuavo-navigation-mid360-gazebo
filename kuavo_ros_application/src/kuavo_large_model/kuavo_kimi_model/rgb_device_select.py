import cv2
import os
import subprocess
import glob

# 设置OpenCV后端，避免Qt显示问题
os.environ['QT_QPA_PLATFORM'] = 'offscreen'

class CameraManager:
    def __init__(self):
        # self.max_index = max_index
        self.rgb_camera_index = None
        
    def get_camera_info(self):
        """获取所有摄像头设备的详细信息"""
        devices = []
        try:
            # 使用v4l2-ctl获取设备信息
            result = subprocess.run(['v4l2-ctl', '--list-devices'], 
                                  capture_output=True, text=True)
            if result.returncode == 0:
                print("=== 系统摄像头设备信息 ===")
                print(result.stdout)
            
            # 检查每个video设备的格式支持
            video_devices = sorted(glob.glob('/dev/video*'))
            
            for device in video_devices:
                device_num = device.split('video')[-1]
                try:
                    # 获取设备支持的格式
                    result = subprocess.run(['v4l2-ctl', '-d', device, '--list-formats-ext'], 
                                          capture_output=True, text=True)
                    if result.returncode == 0:
                        print(f"\n {device} 支持的格式:")
                        formats = result.stdout
                        print(formats)
                        
                        # 分析格式判断是否为RGB相机
                        is_rgb = any(fmt in formats.upper() for fmt in ['YUYV', 'MJPG', 'RGB', 'BGR'])
                        is_depth = any(fmt in formats.upper() for fmt in ['Z16', 'Y16', 'DEPTH'])
                        
                        devices.append({
                            'device': device,
                            'index': int(device_num),
                            'formats': formats,
                            'is_rgb': is_rgb,
                            'is_depth': is_depth
                        })
                        
                except Exception as e:
                    print(f"无法获取 {device} 信息: {e}")
                    
        except FileNotFoundError:
            print("v4l2-ctl 未安装，建议安装: sudo apt install v4l-utils")
        
        return devices

    def find_rgb_camera_by_format(self):
        """基于设备格式查找RGB相机"""
        print("正在基于设备格式检测RGB相机...")
        
        # 获取设备信息
        devices = self.get_camera_info()
        
        # RGB相机格式优先级（按优先级排序）
        rgb_format_priorities = [
            # 优先级1：纯RGB格式，无深度格式
            {'required': ['MJPG'], 'forbidden': ['Z16', 'Y16', 'DEPTH', 'IR'], 'priority': 1},
            {'required': ['YUYV'], 'forbidden': ['Z16', 'Y16', 'DEPTH', 'IR'], 'priority': 2},
            
            # 优先级2：混合设备中的RGB格式（可能包含深度，但有RGB）
            {'required': ['YUYV'], 'forbidden': [], 'priority': 3},
            {'required': ['MJPG'], 'forbidden': [], 'priority': 4},
        ]
        
        # 按优先级整理候选设备
        candidates = []
        
        for device_info in devices:
            device_path = device_info['device']
            index = device_info['index']
            formats = device_info['formats'].upper()
            
            print(f"\n分析设备 {device_path}:")
            print(f"  设备格式: {formats.strip()}")
            
            # 检查每个优先级规则
            for rule in rgb_format_priorities:
                # 检查必需格式
                has_required = any(fmt in formats for fmt in rule['required'])
                # 检查禁止格式
                has_forbidden = any(fmt in formats for fmt in rule['forbidden'])
                
                if has_required and not has_forbidden:
                    print(f"  ✓ 匹配RGB规则(优先级{rule['priority']}): 包含{rule['required']}, 排除{rule['forbidden']}")
                    candidates.append((index, rule['priority']))
                    break
                elif has_required and has_forbidden:
                    print(f"  ⚠ 部分匹配: 包含{rule['required']}但也包含{rule['forbidden']}")
                    if not rule['forbidden']:  # 如果规则允许混合格式
                        candidates.append((index, rule['priority']))
                        break
        
        if not candidates:
            print("\n未找到任何RGB格式的设备")
            return None
        
        # 按优先级排序候选设备
        candidates.sort(key=lambda x: x[1])  # 按优先级排序（数字越小优先级越高）
        
        print(f"\n找到 {len(candidates)} 个候选RGB设备，选择优先级最高的设备")
        
        # 返回优先级最高的设备
        best_camera_index = candidates[0][0]
        print(f"✓ 选择RGB相机: /dev/video{best_camera_index}")
        return best_camera_index


    def get_rgb_camera_index(self):
        """获取RGB相机索引"""
        if self.rgb_camera_index is not None:
            return self.rgb_camera_index
            
        print("开始检测RGB摄像头...")
        
        # 使用基于格式的检测方法
        camera_index = self.find_rgb_camera_by_format()
        if camera_index is not None:
            self.rgb_camera_index = camera_index
            return camera_index
        
        
        print("未检测到可用摄像头")
        return None

# ===== 使用示例 =====
if __name__ == "__main__":
    import logging
    
    logging.basicConfig()
    app_id = "86fd6286"  # 替换为实际的app_id
    api_key = "acf0bf5718c736ccbf132cf815d99cb3"  # 替换为实际的api_key
    
    cam_manager = CameraManager()
    rgb_index = cam_manager.get_rgb_camera_index()
    
    if rgb_index is not None:
        print(f"\n检测到RGB摄像头索引: {rgb_index}")
        cap = cv2.VideoCapture(rgb_index)  # 替换为实际的摄像头设备索引
        
        if cap.isOpened():
            print("成功打开RGB摄像头")
            cap.release()
        else:
            print("无法打开RGB摄像头")
    else:
        print("未找到RGB摄像头")