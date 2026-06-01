# 机器人关节标定一键启动脚本使用说明

## 📋 脚本功能概述

一键启动脚本 `One_button_start.sh` 自动化了机器人关节标定的完整流程：

1. **虚拟环境安装**：安装标定所需的虚拟环境
2. **下位机准备**：准备机器人控制系统
3. **上位机启动**：启动AprilTag识别系统
4. **头部标定**：自动运行头部关节标定，并保存标定结果
5. **手臂标定**：自动运行手臂关节标定，支持左手、右手或双手标定，并保存标定结果

## 🚀 使用方法

### 前置准备

1. **安装标定工具**：
   - 在机器人躯干上安装3D打印的标定工具
   - 在标定工具上正确贴好AprilTag（ID=0）

2. **确保网络连接**：
   - 下位机能SSH连接到上位机192.168.26.1
   - 上位机用户名：kuavo，密码：leju_kuavo
   - **重要**：上位机root密码也是：leju_kuavo（用于进程清理）

3. **上位机中 kuavo_ros_application 仓库**
   - 在脚本执行之前编译
   - 修改 `src/dynamic_biped/launch/sensor_apriltag_only_enable.launch` 

   ```yaml
   <!-- sensor_apriltag_only_enable.launch -->
   <launch>
       <!-- Launch realsense2_camera for head camera -->
       <include file="$(find realsense2_camera)/launch/rs_camera.launch" >
           <arg name="color_width"   value="640"/>
           <arg name="color_height"  value="480"/>
           <arg name="color_fps"     value="30"/>
           <arg name="depth_width"   value="848"/>
           <arg name="depth_height"  value="480"/>
           <arg name="depth_fps"     value="30"/>
           <arg name="enable_infra"        default="false"/>
           <arg name="enable_infra1"       default="false"/>
           <arg name="enable_infra2"       default="false"/>
           <arg name="enable_sync"   value="true"/>
           <arg name="align_depth"   value="true"/>
           <arg name="enable_pointcloud"   value="true"/>
       </include>
       <!-- tf2_ros 静态转换 发布urdf里面的head_camera 和 相机坐标系下的camera_link 进行对齐 -->
       <node pkg="tf2_ros" type="static_transform_publisher" name="camera_to_real_frame" args="0 0 0 0 0 0 camera camera_link" />
       <node pkg="tf2_ros" type="static_transform_publisher" name="base_link_to_torso" args="0 0 0 0 0 0 base_link torso" />
       <!-- 启动 apriltag_ros continuous_detection -->
       <include file="$(find apriltag_ros)/launch/continuous_detection.launch">
         <arg name="camera_name" value="/camera/color" />
         <arg name="image_topic" value="image_raw" />
       </include>
       <!-- 启动 ARControlNode -->
       <node pkg="ar_control" type="ar_control_node.py" name="ar_control_node" output="screen" respawn="true" respawn_delay="5" />
   </launch>

   ```

   + 修改 `src/ros_vision/detection_apriltag/apriltag_ros/config/tags.yaml`，将tag的size尺寸修改为和立方体tag码的尺寸一致

   ```yaml
   standalone_tags:
     [
       {id: 0, size: 0.088, name: 'tag_0'},
       {id: 1, size: 0.088, name: 'tag_1'},
       {id: 2, size: 0.088, name: 'tag_2'},
       {id: 3, size: 0.088, name: 'tag_3'},
       {id: 4, size: 0.088, name: 'tag_4'},
       {id: 5, size: 0.088, name: 'tag_5'},
       {id: 6, size: 0.088, name: 'tag_6'},
       {id: 7, size: 0.088, name: 'tag_7'},
       {id: 8, size: 0.088, name: 'tag_8'},
       {id: 9, size: 0.088, name: 'tag_9'},
     ]
   ```


### 运行一键启动脚本

```bash
# 以root权限运行（需要启动roscore和SSH连接）
sudo su
./scripts/joint_cali/One_button_start.sh
```

### 脚本执行流程

#### 步骤1-3：系统启动
脚本会自动：
- 安装标定所需的虚拟环境
- 启动下位机launch文件（同时就会启动roscore）
- 连接上位机并启动AprilTag识别系统


#### 步骤4：头部标定
脚本会：
1. 检查虚拟环境和配置文件
2. 显示当前标定配置
3. 询问是否继续头部标定
4. 如果选择继续，自动运行头部标定脚本

**用户交互**：
- 脚本会询问：`是否继续头部标定？(y/N)`
- 输入 `y` 或 `Y` 继续，其他任何输入跳过
- 头部标定完成后，脚本会提示：`按下回车键继续保存文件，或者ctrl+c退出`
  - 按 **Enter** 保存校准结果
  - 按 **Ctrl+C** 放弃校准结果

#### 步骤5：手臂标定
脚本会：
1. 检查手臂标定环境和配置文件
2. 询问是否进行手臂标定，以及选择标定哪只手（左手、右手或双手）
3. 如果选择继续，自动运行手臂标定脚本 `arm_cail_noui.py`

**标定流程**：
- 启用机器人移动功能
- 启用头部追踪
- 执行手臂示教运动（播放rosbag文件）
- 在运动过程中收集AprilTag位姿数据
- 过滤噪声数据
- 执行标定算法计算关节偏置
- 提示用户确认是否保存标定结果

**用户交互**：
- 标定完成后，系统会显示标定结果并询问：
  ```
  标定已完成，是否应用新的零点位置？
  输入选项:
    y/yes - 确认并保存标定结果
    n/no  - 取消保存（默认选项）
  ```
- 输入 `y` 或 `yes` 保存标定结果
- 输入 `n`、`no` 或直接按 **Enter** 放弃标定结果

**运行参数**：
手臂标定支持以下参数：
```bash
# 使用自定义参数运行手臂标定
python3 ./scripts/joint_cali/arm_cail_noui.py --side=l --size=20 --real --no-confirm
```
- `--side`: 选择标定哪只手 (l=左手, r=右手, both=双手)
- `--size`: 每只手收集的数据点数量
- `--real`: 实物模式（不加此参数为仿真模式）
- `--no-confirm`: 跳过用户确认，适用于自动化环境


## ⚠️ 注意事项

1. **权限问题**：
   - 脚本需要sudo权限来启动roscore和SSH连接
   - 上位机需要root权限清理进程，确保root密码设置正确
2. **网络连接**：确保下位机能正常连接上位机
3. **标定工具**：确保标定工具正确安装且AprilTag可被相机识别
4. **环境依赖**：确保虚拟环境已正确安装（运行 `create_venv.sh`）
5. **密码配置**：上位机用户密码和root密码都是 `leju_kuavo`
6. **AprilTag检测**：确保相机能稳定检测到AprilTag，光照条件适宜
7. **rosbag文件**：确保手臂标定所需的rosbag文件位于正确位置
   - `./scripts/joint_cali/bags/hand_move_demo_left.bag`
   - `./scripts/joint_cali/bags/hand_move_demo_right.bag`

## 🛠️ 故障排除

### 常见问题

1. **SSH连接失败**：
   - 检查网络连接
   - 确认上位机IP地址和密码

2. **AprilTag识别失败**：
   - 检查相机是否正常工作
   - 确认AprilTag是否在相机视野内
   - 检查光照条件

3. **虚拟环境问题**：
   - 运行 `sudo bash scripts/joint_cali/create_venv.sh` 重新安装

4. **标定数据不足**：
   - 如果收集到的数据点数量为0，检查AprilTag是否被正确检测
   - 检查头部追踪是否正常工作
   - 调整rosbag播放速率（使用 `--rate` 参数）

5. **手臂标定失败**：
   - 检查rosbag文件是否存在且可播放
   - 确认标定环境中的机器人位置与录制rosbag时一致
   - 检查AprilTag立方体是否稳固安装

6. **第二次标定失败**
   - 若是出现 `跳过机器人控制系统启动`，需要开启新终端执行标定

7. **标定结束**
   - 运行完一次标定后，需要在 `screen -r robot_control` 中结束程序运行
   - 第二次标定卡在 `是否继续头部标定？(y/N):`，需要重启机器


## 📝 开发说明

- 脚本设计为模块化，易于维护和扩展
- 手臂标定脚本 `arm_cail_noui.py` 是 `arm_cali.py` 的无GUI版本
- 可以通过修改配置文件调整标定参数
- 支持非交互式环境运行，适用于自动化测试和批量标定 