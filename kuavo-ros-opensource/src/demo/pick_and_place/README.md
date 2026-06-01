# KUAVO Pick and Place Demo

## 概述

`pick_and_place` 包提供了基于视觉引导的物体抓取和放置演示程序。该程序使用 ArUco 标记进行目标检测，支持头部相机和手腕相机的视觉反馈，并实现了精确的位置控制。

## 前提条件

### 手眼标定
在使用该演示程序之前，请确保完成相应相机的手眼标定。标定方法请参考：[KUAVO手眼标定文档](../../hand_eye_calibration/kuavo_hand_eye_calibration/README.md)

### 其他依赖
- ROS（机器人操作系统）
- ArUco ROS包
- KUAVO-ROS-CONTROL机器人包
- RGB相机（RealSense相机）

## 启动方法

1. 启动机器人，直到机器人站立

2. 启动相机节点
```bash
# 头部相机
roslaunch realsense2_camera rs_camera.launch camera:=head_camera 

# 右手手腕（如果使用）
roslaunch realsense2_camera rs_camera.launch camera:=right_wrist_camera

# 左手手腕（如果使用）
roslaunch realsense2_camera rs_camera.launch camera:=left_wrist_camera
```

3. 启动抓取演示程序
```bash
export DISPLAY=:1.0  # 在机器人实物上需要设置
source <kuavo-ros-control>/devel/setup.bash
roslaunch pick_and_place pick_and_place.launch
```

## 参数配置

### 启动文件参数

在 `pick_and_place.launch` 中可以配置以下参数：

- `has_head`：是否使用头部相机（默认：true）
- `control_hand_side`：控制使用哪只手臂（0：左手，1：右手）
- `wrist_side`：使用哪只手腕相机（left：左手，right：右手）
- `has_wrist`：是否使用手腕相机（默认：true）

### 位姿偏移参数

在 `config/offsets.yaml` 中可以配置抓取和放置的位姿偏移：

```yaml
offsets:
  pick:
    grasp_x: 0.0    # 抓取位置X轴偏移
    grasp_y: 0.0    # 抓取位置Y轴偏移
    grasp_z: 0.05   # 抓取位置Z轴偏移
  place:
    place_x: 0.0    # 放置位置X轴偏移
    place_y: 0.0    # 放置位置Y轴偏移
    place_z: 0.05   # 放置位置Z轴偏移

orientation:
  pick:
    roll: 0.0       # 抓取姿态roll角度（度）
    pitch: 90.0     # 抓取姿态pitch角度（度）
    yaw: 0.0        # 抓取姿态yaw角度（度）
  place:
    roll: 0.0       # 放置姿态roll角度（度）
    pitch: 90.0     # 放置姿态pitch角度（度）
    yaw: 0.0        # 放置姿态yaw角度（度）
```

## 使用说明

1. 准备 ArUco 标记
   - 物体标记：使用 ID 为 776 的 ArUco 标记 [下载PDF](assets/aruco-776.pdf)
   - 放置位置标记：使用 ID 为 777 的 ArUco 标记 [下载PDF](assets/aruco-777.pdf)
   - 标记尺寸默认为 5cm × 5cm
   - 请确保打印尺寸正确，建议使用硬质材料打印并固定

2. 自定义 ArUco 标记参数（如需要）
   - 修改 launch 文件中的参数 (`pick_and_place.launch`):
     ```xml
     <arg name="marker_id1" default="776" />    <!-- 物体标记ID -->
     <arg name="marker_id2" default="777" />    <!-- 放置位置标记ID -->
     <arg name="marker_size" default="0.05" />  <!-- 标记边长(米) -->
     ```

   - 同时需要修改配置文件 (`config/markers.yaml`):
     ```yaml
     marker_object_frame:
       marker_id: 776          # 物体标记ID
       marker_size: 0.05       # 标记边长(米)

     marker_place_frame:
       marker_id: 777          # 放置位置标记ID
       marker_size: 0.05       # 标记边长(米)
     ```
   
   注意：两个文件中的参数需要保持一致

3. 程序启动后会进入交互模式
4. 按照提示按回车键执行每个步骤
5. 检测失败时可以重复尝试，按 'q' 退出
6. 支持手腕相机和头部相机的自动切换

## 注意事项

1. 确保 ArUco 标记清晰可见
2. 标记应放置在机器人工作范围内
3. 避免标记被遮挡或强光干扰
4. 检查相机是否正确标定
5. 确保机器人和相机通信正常

## 故障排除

1. **标记检测失败**
   - 检查光照条件
   - 确认标记在相机视野内
   - 验证标记 ID 是否正确

2. **位置精度不足**
   - 检查手眼标定结果
   - 确认机器人状态正常

3. **机器人运动异常**
   - 检查工作空间限制
   - 确认 IK 解算正常
   - 验证安全设置是否正确

## TIPS

### 标记放置建议

- 使用右臂进行抓取时：
  - 标记的高度（Z轴）在 base_link 坐标系下应高于 0.3 米
  - 标记到机器人的距离（X轴）在 base_link 坐标系下应小于 0.5 米
  - 这样可以确保标记在机器人的最佳工作范围内，提高抓取成功率 