# kuavo_audio_receiver

**麦克风音频接收器 ROS 包**

本包用于接收麦克风音频数据并发布到 ROS 话题，支持多种机型的 USB 麦克风设备。

## 🎯 功能特性

- **实时音频捕获**: 连续监听麦克风并发布音频数据
- **自动设备识别**: 智能识别指定的 USB 麦克风设备
- **降采样处理**: 自动将音频降采样到 16kHz 目标频率
- **多机型兼容**: 支持不同型号的 Kuavo 机器人
- **错误处理**: 完善的错误处理和优雅关闭机制

## 📦 系统要求

### ROS 依赖
- `rospy`
- `kuavo_msgs`

### Python 依赖
- `pyaudio` - 音频输入/输出处理
- `numpy` - 数值计算
- `scipy` - 信号处理  
- `samplerate` - 高质量音频重采样

### 系统依赖
- `portaudio19-dev` - PortAudio 开发库
- `python3-pyaudio` - PyAudio Python 绑定

## 🚀 安装与配置

### 1. 构建消息类型
```bash
catkin_make --pkg kuavo_msgs
```

### 2. 构建音频接收器包
```bash
catkin_make --pkg kuavo_audio_receiver
```

### 3. 设置硬件驱动（可选）
```bash
sudo ./tools/load_microphone_driver/load_microphone_driver.sh
```

## 📡 接口规范

### 发布话题
- **话题名称**: `/microphone_data`
- **消息类型**: `kuavo_msgs/AudioReceiverData`
- **消息格式**: `uint8[] data` - 原始音频字节数据

### 音频规格
- **采样率**: 16kHz（降采样后）
- **位深**: 16-bit
- **声道**: 单声道
- **格式**: PCM 有符号整数

## 🎮 使用方法

### 基本启动
```bash
# 启动音频接收器节点
roslaunch kuavo_audio_receiver receive_voice.launch
```

### 手动启动节点
```bash
# 直接运行 Python 脚本
rosrun kuavo_audio_receiver micphone_receiver_node.py
```

### 验证音频数据
```bash
# 监听音频话题
rostopic echo /microphone_data

# 检查话题信息
rostopic info /microphone_data

# 查看消息频率
rostopic hz /microphone_data
```

## ⚙️ 配置参数

### 支持的麦克风设备
- **Jieli Technology** - 杰理科技 USB 麦克风
- **USB Composite Device** - 通用 USB 复合设备

### 硬件配置
- **目标 VendorID**: `4c4a`
- **目标 ProductID**: `4155`  
- **缓冲区大小**: 1024 帧
- **降采样算法**: `sinc_fastest`
