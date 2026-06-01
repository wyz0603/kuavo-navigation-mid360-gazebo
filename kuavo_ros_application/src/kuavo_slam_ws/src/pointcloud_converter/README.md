# Point Cloud Converter

这个包提供了一个中间转换节点，用于将Hesai激光雷达的点云格式转换为与FAST_LIO兼容的Velodyne格式。

## 功能特性

- **格式转换**: 将Hesai点云格式（timestamp字段）转换为Velodyne格式（time字段）
- **数据类型转换**: 将双精度时间戳转换为单精度相对时间
- **点云滤波**: 支持距离和强度滤波
- **统计信息**: 提供转换统计和性能监控
- **参数化配置**: 通过YAML文件和launch参数灵活配置

## 安装和编译

```bash
cd /home/zhongxu/work/kuavo_ros_application
catkin build pointcloud_converter
source devel/setup.bash
```

## 使用方法

### 1. 基本转换器测试

```bash
# 启动转换器（需要先启动Hesai驱动）
roslaunch pointcloud_converter hesai_converter.launch

# 或者指定自定义话题
roslaunch pointcloud_converter hesai_converter.launch input_topic:=/your_input_topic output_topic:=/your_output_topic
```

### 2. 集成建图

```bash
# 使用转换器进行建图
roslaunch pointcloud_converter build_map_with_converter.launch
```

### 3. 测试转换器功能

```bash
# 启动测试环境
roslaunch pointcloud_converter test_converter.launch
```

## 配置参数

### 话题配置
- `input_topic`: Hesai激光雷达输入话题 (默认: `/lidar_points`)
- `output_topic`: 转换后的输出话题 (默认: `/velodyne_points`)

### 坐标系配置
- `input_frame`: 输入点云坐标系 (默认: `hesai_lidar`)
- `output_frame`: 输出点云坐标系 (默认: `velodyne`)

### 转换配置
- `time_scale_factor`: 时间戳缩放因子 (默认: 1000.0，秒转毫秒)

### 滤波配置
- `use_range_filter`: 启用距离滤波 (默认: true)
- `min_range`: 最小距离 (默认: 0.5m)
- `max_range`: 最大距离 (默认: 100.0m)
- `use_intensity_filter`: 启用强度滤波 (默认: false)
- `min_intensity`: 最小强度 (默认: 0.0)
- `max_intensity`: 最大强度 (默认: 255.0)

## 点云格式对比

### Hesai格式 (输入)
```
fields:
  - name: x, type: FLOAT32
  - name: y, type: FLOAT32
  - name: z, type: FLOAT32
  - name: intensity, type: FLOAT32
  - name: timestamp, type: FLOAT64  # 双精度时间戳
  - name: ring, type: UINT16
```

### Velodyne格式 (输出)
```
fields:
  - name: x, type: FLOAT32
  - name: y, type: FLOAT32
  - name: z, type: FLOAT32
  - name: intensity, type: FLOAT32
  - name: time, type: FLOAT32       # 单精度相对时间
  - name: ring, type: UINT16
```

## 监控和调试

### 查看转换统计
转换器每10秒会输出统计信息：
```
[INFO] Conversion Stats - Frames: 100, Avg Input: 65536.0 pts, Avg Output: 62341.2 pts, Retention: 95.1%
```

### 检查话题
```bash
# 查看输入话题信息
rostopic info /lidar_points

# 查看输出话题信息
rostopic info /velodyne_points

# 查看点云字段
rostopic echo /velodyne_points/fields
```

### 性能监控
```bash
# 查看话题频率
rostopic hz /lidar_points
rostopic hz /velodyne_points

# 查看节点信息
rosnode info /hesai_to_velodyne_converter
```

## 故障排除

### 常见问题

1. **"Input point cloud missing required fields"**
   - 检查输入点云是否包含`timestamp`和`ring`字段
   - 确认Hesai驱动正确配置和运行

2. **"All points filtered out"**
   - 检查滤波参数设置
   - 调整`min_range`和`max_range`参数

3. **转换后点云为空**
   - 检查输入话题是否有数据：`rostopic echo /lidar_points -n 1`
   - 确认坐标系和时间戳设置正确

### 调试模式
```bash
# 启用调试输出
roslaunch pointcloud_converter hesai_converter.launch --screen
```

## 与FAST_LIO集成

转换器输出的点云格式完全兼容FAST_LIO的Velodyne处理器，可以直接用于：
- SLAM建图
- 定位
- 点云处理

确保在FAST_LIO配置中设置：
```yaml
preprocess:
    lidar_type: 2  # Velodyne类型
    scan_line: 128 # 根据您的Hesai激光雷达线数调整
```
