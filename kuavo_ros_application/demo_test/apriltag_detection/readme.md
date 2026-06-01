# AprilTag 视觉检测程序

## 功能概述
本程序通过 RealSense D435 相机和 apriltag_ros 实现 AprilTag 二维码的检测，并输出标签的三维位置信息到apriltag_results.txt文件中。

## 快速开始
1. 环境准备：
   ```bash
   chmod +x start_apriltag_detection.sh
   ./start_apriltag_detection.sh
   ```

2. 远程桌面配置（如需要）：
   ```bash
   # 在 start_apriltag_detection.sh 中取消以下注释
   # export XDG_RUNTIME_DIR="/tmp/runtime-$USER"
   # mkdir -p "$XDG_RUNTIME_DIR"
   # chmod 700 "$XDG_RUNTIME_DIR"
   # export DISPLAY=:1
   ```

## AprilTag 标签准备
1. 获取方式：
   - 使用[在线生成工具](https://chev.me/arucogen/)
   - 选择类型：apriltag (36h11)


## 核心组件

### 1. 标签信息处理器 (get_tag_info.py)
主要功能：
- 实时获取标签位置和姿态信息
- 支持单次和多次采样数据获取
- 提供四元数到欧拉角的转换

关键方法：
```python
get_apriltag_data()          # 获取所有可见标签数据
get_apriltag_by_id(tag_id)   # 获取指定ID标签数据
get_averaged_apriltag_data(tag_id, num_samples=10)  # 获取平均数据
```

数据格式：
```python
{
    "id": 标签ID,
    "off_horizontal": x偏移(米),
    "off_camera": y偏移(米),
    "off_vertical": z偏移(米),
    "roll_angle": 横滚角(-180° ~ 180°),
    "pitch_angle": 俯仰角(-180° ~ 180°),
    "yaw_angle": 偏航角(-180° ~ 180°)
}
```

### 2. 启动脚本 (start_apriltag_detection.sh)
功能：
- 清理已有 ROS 进程
- 部署标签配置文件
- 启动相机和检测节点
- 运行位置检测程序

### 3. 标签配置 (tags.yaml)
```yaml
standalone_tags:
  [
    {id: 0, size: 0.093, name: 'tag_0'},  # 尺寸单位：米
    {id: 1, size: 0.042, name: 'tag_1'},
  ]
```

## 输出说明
程序运行后会在 `apriltag_results.txt` 中保存检测结果：
- 标签 ID
- 三维位置偏移（米）
- 偏航角度（度）

## 注意事项
1. 确保 RealSense D435 相机正确连接
2. 标签尺寸配置单位为米
3. 使用 Ctrl+C 可正常停止所有节点
4. 打印标签时需精确测量实际尺寸
5. 确保运行脚本前标签放置在相机可见范围内

