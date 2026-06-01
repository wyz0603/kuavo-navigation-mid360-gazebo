# 单按键设备监听器

专门用于监听 Vendor ID: 0x4132, Product ID: 0x2107 的单按键USB设备。

## 设备独有特征

您的设备具有以下独特特征，这些可以用来精确识别和监听：

### 硬件标识
- **Vendor ID**: 0x4132 (16690)
- **Product ID**: 0x2107 (8455)
- **USB版本**: 1.10
- **功耗**: 100mA

### 接口配置（独有特征）
**4个HID接口组合** - 这是比较少见的配置：
1. **接口0**: HID v1.11 键盘 (端点0x81, 8字节包)
2. **接口1**: HID v1.10 鼠标 (端点0x82, 8字节包)  
3. **接口2**: HID v1.10 通用设备 (端点0x83, 8字节包)
4. **接口3**: HID v1.00 双向通信 (端点0x84/0x04, **64字节包**) ⭐

> **关键特征**: 4个HID接口的组合在USB设备中很少见，可以作为独特的设备指纹

### 按键检测特征
- **状态报告**: 通过hidraw接口的0x29报告检测按键状态
- **状态值**: 0x01 (按下) / 0x00 (释放)
- **数据格式**: 64字节包，前2字节为[0x29, 状态]，其余62字节为0
- **智能识别**: 自动监听所有设备，直到收到0x29报告才确定目标设备

## 安装与编译

### 编译ROS包

```bash
# 进入工作空间根目录
cd <kuavo_ros_application>

catkin build kuavo_audio_single_button

source devel/setup.bash
```

## 使用方法

### 直接运行（推荐）

现在脚本会自动查找和识别设备，无需预先配置：

#### 单按键检测器（独立运行）

```bash
sudo python3 scripts/button_detector.py
```

#### ROS节点运行

```bash
# 确保已经source了工作空间
source <kuavo_ros_application>/devel/setup.bash

# 需要先切换到root用户
sudo su

# 启动ROS节点
rosrun kuavo_audio_single_button button_detector_ros_node.py

# 在另一个终端查看发布的话题
rostopic echo /kuavo/audio_single_button/event
```

### 查看帮助
```bash
python3 scripts/button_detector.py --help
```

## 按键检测功能

### 检测能力
- **按键按下/释放**: 实时检测按键状态变化
- **持续时间计算**: 精确计算按键持续时间
- **按键类型识别**: 
  - 快速点击 (< 0.2秒)
  - 正常按键 (0.2-1.0秒)
  - 长按 (> 1.0秒)

### 设备识别
- **自动匹配**: 通过vendor:product ID + 4接口组合验证
- **智能监听**: 同时监听所有HID接口，等待0x29报告
- **动态识别**: 收到0x29报告时自动确定目标设备
- **无需配置**: 无需预先配置，即插即用

## 示例输出

```bash
# 设备查找和识别
正在搜索设备 4132:2107...
独有特征：4个HID接口组合
找到USB设备: Bus 003 Device 011: ID 4132:2107
找到HID接口: /dev/hidraw0
找到HID接口: /dev/hidraw1
找到HID接口: /dev/hidraw2
找到HID接口: /dev/hidraw3
✅ 验证通过：找到4个HID接口 - ['/dev/hidraw0', '/dev/hidraw1', '/dev/hidraw2', '/dev/hidraw3']

# 智能设备识别
正在等待按键事件来识别目标设备...
🎯 找到按键设备: /dev/hidraw3
   报告ID: 0x29
   按键状态: 0x01

# 按键检测
🔴 [11:13:02] 按键按下 (来自: /dev/hidraw3)
🔵 [11:13:02] 按键释放 (来自: /dev/hidraw3) - 持续: 0.220秒 (正常按键)
```

## 重要提示

1. **需要root权限**: 访问 `/dev/hidraw*` 设备文件
2. **设备识别**: 通过4个HID接口组合精确识别设备
3. **状态监听**: 基于0x29状态报告进行按键检测
4. **实时响应**: 毫秒级按键检测精度
5. **智能识别**: 自动监听所有设备，等待0x29报告确定目标设备
6. **即插即用**: 无需预先配置，设备连接后直接运行即可

## 故障排除

1. **找不到包或节点**:
   - 确保已经编译包: `catkin build kuavo_audio_single_button`
   - 确保已经source环境: `source devel/setup.bash`
2. **权限错误**: 
   - 确保使用 `sudo su` 切换到root用户
   - 切换用户后记得重新source环境
3. **设备未找到**: 
   - 检查设备是否已连接: `lsusb | grep 4132:2107`
   - 确认4个HID接口存在: `ls /sys/class/hidraw/`
4. **无按键响应**: 
   - 确保设备已连接并按下按键
   - 脚本会自动监听所有设备直到收到0x29报告
   - 查看输出中的"🎯 找到按键设备"信息
5. **设备重新插拔后失效**:
   - 重新运行脚本，会自动重新识别设备
   - 无需任何额外配置

## ROS集成

### ROS节点功能

该包提供了一个ROS节点 `button_detector_ros_node.py`，用于发布按键事件到ROS话题。

#### 发布话题

- **话题名称**: `/kuavo/audio_single_button/event`
- **消息类型**: `std_msgs/Bool`
- **消息值**:
  - `True`: 按键按下
  - `False`: 按键释放

#### 使用示例

```bash
# 在一个终端切换到root用户并启动节点
sudo su
rosrun kuavo_audio_single_button button_detector_ros_node.py

# 在另一个终端监听话题
rostopic echo /kuavo/audio_single_button/event

# 查看话题信息
rostopic info /kuavo/audio_single_button/event
```

### 应用集成示例

可以将按键检测集成到其他应用中：

- **ROS节点**: 订阅 `/kuavo/audio_single_button/event` 话题获取按键事件
- **系统快捷键**: 绑定特定功能
- **数据记录**: 记录按键使用统计
- **设备控制**: 基于按键控制其他设备

#### Python订阅示例

```python
import rospy
from std_msgs.msg import Bool

def button_callback(msg):
    if msg.data:
        print("按键按下")
    else:
        print("按键释放")

rospy.init_node('button_listener')
rospy.Subscriber('/kuavo/audio_single_button/event', Bool, button_callback)
rospy.spin()
```

## 脚本说明

### button_detector.py
标准按键检测器，功能包括：
- 自动查找所有hidraw设备
- 智能监听所有设备直到收到0x29报告
- 动态识别目标设备
- 实时按键检测和分类
- 无需预先配置

### button_detector_ros_node.py
ROS节点版本，功能包括：
- 发布按键事件到ROS话题
- 自动查找所有hidraw设备
- 智能监听所有设备直到收到0x29报告
- 动态识别目标设备
- 无需预先配置
