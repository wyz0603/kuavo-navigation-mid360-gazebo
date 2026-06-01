# 快速使用指南

## 🚀 快速开始

### 1. 编译转换器
```bash
cd /home/zhongxu/work/kuavo_ros_application
catkin build pointcloud_converter
source devel/setup.bash
```

### 2. 使用转换器进行建图
```bash
# 方法1: 使用集成的建图启动文件（推荐）
roslaunch pointcloud_converter build_map_with_converter.launch

# 方法2: 分步启动
# 终端1: 启动Hesai驱动
roslaunch hesai_ros_driver start.launch

# 终端2: 启动转换器
roslaunch pointcloud_converter hesai_converter.launch

# 终端3: 启动FAST_LIO（使用转换后的点云）
roslaunch fast_lio mapping_jt128.launch
```

### 3. 验证转换效果
```bash
# 检查输入点云字段
rostopic echo /lidar_points/fields

# 检查输出点云字段
rostopic echo /velodyne_points/fields

# 查看转换统计
rostopic echo /rosout | grep "Conversion Stats"
```

## 🔧 解决的问题

这个转换器解决了以下问题：
- ✅ "Failed to find match for field 'time'" → 将`timestamp`字段转换为`time`字段
- ✅ "Failed to find match for field 'ring'" → 保持`ring`字段兼容
- ✅ "No Effective Points!" → 确保点云数据正确处理
- ✅ 数据类型不匹配 → 将双精度时间戳转换为单精度

## 📊 预期结果

转换器运行后，您应该看到：
```
[INFO] Hesai to Velodyne Converter initialized
[INFO] Input topic: /lidar_points
[INFO] Output topic: /velodyne_points
[INFO] Conversion Stats - Frames: 50, Avg Input: 65536.0 pts, Avg Output: 62341.2 pts, Retention: 95.1%
```

FAST_LIO将不再报错，并能正常处理点云数据进行建图。
