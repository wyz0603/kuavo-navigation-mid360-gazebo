# YOLO 视觉检测程序

## 功能概述
本程序通过机器人头部相机和 YOLOv8 模型实现箱子的实时检测，可获取目标的三维位置信息，并输出到yolo_box_info.txt文件中。

## 快速开始
1. 环境准备：
   ```bash
   chmod +x start_detection_box.sh
   ./start_detection_box.sh
   ```

2. 远程桌面配置（如需要）：
   ```bash
   # 在 start_detection_box.sh 中已包含以下配置
   export XDG_RUNTIME_DIR="/tmp/runtime-$USER"
   mkdir -p "$XDG_RUNTIME_DIR"
   chmod 700 "$XDG_RUNTIME_DIR"
   export DISPLAY=:1
   ```

## 模型准备
1. 模型要求：
   - 格式：YOLOv8 (.pt文件)
   - 位置：`<kuavo_ros_application>/src/ros_vision/detection_industrial_yolo/yolo_box_object_detection/scripts/models/weights/best.pt`

2. 检测对象：
   - 目标类型：箱子
   - 置信度阈值：0.6

## 核心组件

### 1. 位置信息处理器 (yolo_detection_average_info.py)
主要功能：
- 实时获取箱子位置和姿态信息
- 支持多次采样数据平均


关键方法：
```python
get_position_and_orientation(sample_count=100)  # 获取平均位置和方向数据
normalize_quaternion(quat)                      # 四元数归一化
save_detection_results()                        # 保存检测结果
```

数据格式：
```python
{
    "position": {
        "x": float,  # 米
        "y": float,  # 米
        "z": float   # 米
    },
    "orientation": {
        "x": float,
        "y": float,
        "z": float,
        "w": float
    }
}
```

### 2. 启动脚本 (start_detection_box.sh)
功能：
- 清理已有 ROS 进程
- 配置显示环境
- 启动相机和检测节点
- 运行位置检测程序

### 3. ROS话题
```bash
/object_yolo_box_segment_result     # 相机坐标系下的位置
/object_yolo_box_segment_image      # 识别结果可视化
/object_yolo_box_tf2_torso_result   # 机器人基坐标系下的位置
```

## 输出说明
程序运行后会在 `yolo_box_info.txt` 中保存检测结果：
- 检测时间
- 三维位置信息（米）


## 注意事项
1. 确保机器人头部相机正确连接
2. 检查YOLO模型文件是否存在于指定路径
3. 使用 Ctrl+C 可正常停止所有节点
4. 确保目标箱子在相机可见范围内
5. 测距有效范围：0.3~3m(实际测试发现超过2m后，测距误差会增大)
6. 测距时不要将相机对着光源，否则测距会不稳定
7. 测距背景尽量单调，确保相机和快递盒之间没有物体遮挡，以及没有其它盒子干扰