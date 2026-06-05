# Kuavo 人形机器人 MID360 激光雷达仿真与自主导航

**语言 / Language：中文 | [English](./README.en.md)**

> 在 Gazebo 仿真中为 Leju Kuavo 人形机器人接入 Livox MID360 激光雷达，打通
> **仿真激光雷达 → FAST‑LIO 里程计/SLAM 建图 → move_base 自主导航** 全链路，
> 并完成导航参数调优与纯追踪（Pure Pursuit）运动控制。

---

## 目录

- [1. 项目简介](#1-项目简介)
- [2. 仓库与原始地址](#2-仓库与原始地址)
- [3. 系统架构与数据流](#3-系统架构与数据流)
- [4. 环境依赖](#4-环境依赖)
- [5. 部署与编译](#5-部署与编译)
- [6. 运行流程](#6-运行流程)
- [7. 测试与验证](#7-测试与验证)
- [8. 关键参数说明](#8-关键参数说明)
- [9. 本项目相对上游的改动清单](#9-本项目相对上游的改动清单)
- [10. 常见问题（FAQ）](#10-常见问题faq)

---

## 1. 项目简介

本项目由两个相互配合的 ROS（Noetic）工作空间组成：

| 工程 | 作用 |
|------|------|
| **kuavo-ros-opensource** | 人形机器人本体 + Gazebo 仿真（MPC‑WBC 控制、8 相步态、Livox MID360 仿真插件，发布 `/scan`、`/livox/imu`） |
| **kuavo_ros_application** | SLAM + 导航栈（FAST‑LIO、octomap、move_base、MPC 局部规划），本仓库 |

核心能力：

- **仿真激光雷达桥接**：C++ 节点 `livox_sim_bridge` 把 Gazebo 插件输出的旧式
  `sensor_msgs/PointCloud`（`/scan`）实时转换为 FAST‑LIO 所需的
  `livox_ros_driver2/CustomMsg` 与导航所需的 `PointCloud2`，并对点云/IMU 做安装姿态补偿。
- **SLAM 建图**：FAST‑LIO 输出 `/Odometry`，octomap_server 生成 2D 占据栅格地图。
- **自主导航**：move_base（全局 Dijkstra/A* + MPC 局部规划）+ 纯追踪节点
  `global_plan_follower`（接管 `/cmd_vel`，被挡时触发清图重规划）。
- **步态联动**：`/cmd_vel` 速度阈值（0.018 m/s）驱动机器人站立 ↔ 行走步态切换。

---

## 2. 仓库与原始地址

本仓库 fork / 迁移自鲸鱼机器人（Leju）官方仓库，原始地址如下：

| 工程 | 原始地址（official） |
|------|------|
| **kuavo-ros-opensource**（机器人本体 + Gazebo 仿真） | **Gitee：https://gitee.com/leju-robot/kuavo-ros-opensource.git** |
| **kuavo_ros_application**（SLAM + 导航） | LejuHub：https://www.lejuhub.com/ros-application-team/kuavo_ros_application.git |

> 📌 **原始 Gitee 仓库**：`https://gitee.com/leju-robot/kuavo-ros-opensource.git`（master 分支）。
> 本项目仅在其基础上增加 MID360 仿真接入与导航集成，本体控制、模型、步态等版权归 Leju Robotics 所有。

---

## 3. 系统架构与数据流

```
┌─────────────────────────── kuavo-ros-opensource (Gazebo) ───────────────────────────┐
│  Livox MID360 仿真插件                 IMU 插件                                       │
│        │ /scan (PointCloud, ~10Hz)         │ /livox/imu (Imu, ~1000Hz)               │
└────────┼───────────────────────────────────┼─────────────────────────────────────────┘
         │                                   │
┌────────▼───────────────────────────────────▼──── kuavo_ros_application ──────────────┐
│  livox_sim_bridge (本项目新增)                                                        │
│     /scan ──► /livox/lidar (CustomMsg)   +  IMU 姿态补偿 ──► /livox/imu_corrected     │
│                         │                                                              │
│                  ┌──────▼──────┐                                                       │
│                  │  FAST-LIO   │ ──► /Odometry  +  /livox/cloud (PointCloud2)          │
│                  └──────┬──────┘            │                                          │
│        ┌────────────────┴────────┐          │                                          │
│   octomap_server            move_base (全局规划 + MPC 局部规划)                        │
│   2D 占据栅格地图                │  global_costmap / local_costmap                      │
│                                  ▼                                                      │
│                       global_plan_follower (Pure Pursuit) ──► /cmd_vel ──► 机器人步态  │
└───────────────────────────────────────────────────────────────────────────────────────┘
```

---

## 4. 环境依赖

- Ubuntu 20.04
- ROS Noetic（`/opt/ros/noetic`）
- Gazebo（随 ROS Noetic 安装）
- `python3-catkin-tools`（`catkin build`）
- 机器人版本环境变量 `ROBOT_VERSION`（如 `45` 代表 4.5 版本）
- 依赖项（首次编译会通过 rosdep / apt 自动安装）：`octovis`、`ros-noetic-tf2-sensor-msgs`、`libyaml-cpp-dev`、`libpcap-dev` 等

```bash
# 设置机器人版本（按实际机器人型号修改，45 = 4.5 版本）
echo 'export ROBOT_VERSION=45' >> ~/.bashrc
source ~/.bashrc
```

---

## 5. 部署与编译

### 5.1 克隆本仓库

本仓库为 **monorepo**，已将两个工作空间作为子目录一并收录：

```bash
git clone https://github.com/wuyangzhi03-dev/kuavo-navigation-mid360-gazebo.git
cd kuavo-navigation-mid360-gazebo
# 子目录：
#   kuavo-ros-opensource/    机器人本体 + Gazebo 仿真
#   kuavo_ros_application/   SLAM + 导航

# 下文命令以 ~/kuavo-ros-opensource、~/kuavo_ros_application 为简写，
# 建立软链接即可让后续命令原样可用：
ln -s "$(pwd)/kuavo-ros-opensource"  ~/kuavo-ros-opensource
ln -s "$(pwd)/kuavo_ros_application" ~/kuavo_ros_application
```

> 机器人本体子目录源自 Leju 官方 Gitee 仓库
> `https://gitee.com/leju-robot/kuavo-ros-opensource.git`（详见[第 2 节](#2-仓库与原始地址)）。

### 5.2 ⚠️ 必改项：修正硬编码绝对路径（重要）

`kuavo-ros-opensource` 的仿真启动文件
`src/humanoid-control/humanoid_controllers/launch/load_kuavo_gazebo_sim.launch`
中含有指向开发机的**硬编码绝对路径**，克隆后必须改成你自己的路径，否则 Gazebo 加载激光雷达/世界会失败：

```xml
<!-- 改成你本机 kuavo_ros_application 克隆路径下的 mid360.csv -->
<arg name="livox_csv_file"
     default="$HOME/kuavo_ros_application/src/kuavo_slam_ws/src/livox_laser_simulation/scan_mode/mid360.csv"/>
<!-- 改成你自己的世界文件路径 -->
<arg name="world_name" default="$HOME/world.world"/>
```

同时确认以下参数（默认即为仿真所需）：

```xml
<arg name="use_livox"   default="true"/>   <!-- 在 Gazebo 中加载 MID360 -->
<arg name="livox_topic" default="/scan"/>  <!-- 与 livox_sim_bridge 输入一致 -->
```

### 5.3 编译机器人本体（kuavo-ros-opensource）

```bash
cd ~/kuavo-ros-opensource
sudo su                       # 该工程编译/运行需 root 环境
source /opt/ros/noetic/setup.bash
catkin build                  # 首次编译 cppad 模型约需数分钟
exit
```

### 5.4 编译 SLAM/导航（kuavo_ros_application）

`kuavo_slam_ws` 是一个**独立的 catkin 工作空间**，使用随仓库提供的脚本一键编译
（脚本会自动构建 Livox‑SDK、`livox_ros_driver2`(ROS1)、FAST‑LIO、octomap、本项目的桥接节点等）：

```bash
cd ~/kuavo_ros_application/src/kuavo_slam_ws
source /opt/ros/noetic/setup.bash
./build_kuavo_slam_ws.sh
```

> 单独重新编译本项目改动的两个包（验证用）：
> ```bash
> cd ~/kuavo_ros_application/src/kuavo_slam_ws
> catkin build --workspace "$(pwd)" pointcloud_converter livox_laser_simulation
> ```

---

## 6. 运行流程

> 每个终端都需先 source 对应环境。约定：
> ```bash
> # 机器人本体环境
> source ~/kuavo-ros-opensource/devel/setup.bash
> # 叠加导航环境
> source ~/kuavo_ros_application/src/kuavo_slam_ws/devel/setup.bash --extend
> ```

### 6.1 启动 Gazebo 仿真（终端 1）

```bash
sudo su
source ~/kuavo-ros-opensource/devel/setup.bash
roslaunch humanoid_controllers load_kuavo_gazebo_sim.launch \
    use_livox:=true joystick_type:=bt2pro rviz:=false
```

启动后机器人在 Gazebo 中站立，并发布 `/scan`、`/livox/imu`。

### 6.2 建图（终端 2）

```bash
source ~/kuavo-ros-opensource/devel/setup.bash
source ~/kuavo_ros_application/src/kuavo_slam_ws/devel/setup.bash --extend
roslaunch kuavo_mapping build_map_sim.launch
```

用手柄遥控机器人绕场景走一圈累积地图。建图会启动 `livox_sim_bridge` + FAST‑LIO + octomap。

**保存地图**（地图输出到 `~/maps/<name>.{pcd,pgm,yaml}`）：

```bash
# 通过工程既有的建图服务保存（需 navigation_service 在运行）
rosservice call /save_map_service "map_name: 'sim_map_1'"
```

> 备注：`build_map_sim.launch` 已设 `pcd_save_en=true`，Ctrl‑C 结束建图时 FAST‑LIO 也会
> 落盘点云，可作为兜底。

### 6.3 自主导航（终端 2，先停掉建图）

```bash
source ~/kuavo-ros-opensource/devel/setup.bash
source ~/kuavo_ros_application/src/kuavo_slam_ws/devel/setup.bash --extend
roslaunch kuavo_navigation kuavo_navigation_sim.launch map:=sim_map_1
```

在 RViz 中用 **2D Nav Goal** 发布目标点，机器人沿全局路径自主行走、绕障到达目标。

常用可选参数：

```bash
# 切换局部规划器（默认 mpc_local_planner）
roslaunch kuavo_navigation kuavo_navigation_sim.launch map:=sim_map_1 base_local_planner:=dwa_local_planner
# 关闭纯追踪、让 MPC 直接驱动 /cmd_vel
roslaunch kuavo_navigation kuavo_navigation_sim.launch map:=sim_map_1 use_global_plan_follower:=false
# 录制导航数据 bag
roslaunch kuavo_navigation kuavo_navigation_sim.launch map:=sim_map_1 record_nav_data:=true
```

---

## 7. 测试与验证

### 7.1 话题连通性

```bash
# 仿真侧
rostopic hz /scan          # ≈10 Hz  PointCloud
rostopic hz /livox/imu     # ≈1000 Hz Imu

# 桥接 + SLAM 侧
rostopic hz /livox/lidar   # ≈10 Hz  CustomMsg
rostopic hz /livox/cloud   # ≈10 Hz  PointCloud2
rostopic hz /Odometry      # FAST-LIO 里程计输出（连续无中断）
rostopic echo -n1 /projected_map   # octomap 2D 占据栅格
```

### 7.2 TF 检查

```bash
rosrun rqt_tf_tree rqt_tf_tree
# 确认 livox_link 只有一个父节点（由 robot_state_publisher 经 URDF 链路提供）
```

### 7.3 导航闭环验证

- 在 RViz 发 2D Nav Goal 后，`/cmd_vel` 持续输出（`rostopic echo /cmd_vel`）。
- 机器人在 Gazebo 中走向目标，遇障绕行；被完全挡住时自动 `clear_costmaps` 并重规划。
- 到达目标容差内（xy 0.25 m / yaw 0.20 rad）后停步，步态切回站立。

**成功标准**：FAST‑LIO 无 `no imu/lidar` 警告；octomap 占据栅格随机器人移动累积；
导航阶段 `/cmd_vel` 持续输出并驱动机器人到达目标。

---

## 8. 关键参数说明

仿真链路的核心可调参数（在对应 launch / config 中）：

| 参数 | 默认值 | 作用 |
|------|--------|------|
| `correction_rpy` | `[3.14159, 0.11, 0.0]` | MID360 倒装 + 俯仰补偿，把雷达/IMU 旋正到竖直参考系 |
| `cloud_z_offset` | `1.5`（visual 模式） | 给每个点云点加常量 Z，使地面落在栅格上 |
| `lidar_min_z` | `-1.2` | 过滤机器人脚下的地面“光环”噪声 |
| `0.018 m/s` | — | `/cmd_vel` 线速度阈值，触发站立 ↔ 行走步态切换 |
| local_costmap `width/height` | `6.0 m` | 局部代价地图范围（上游 3.0 → 6.0，给 MPC 足够反应距离） |
| `inflation_radius` | `0.5 m` | 膨胀半径（上游 1.2 → 0.5，避免狭窄通道被高代价填满） |
| `cost_scaling_factor` | `3` | 代价衰减坡度（上游 8 → 3，使 MPC 能感知梯度提前避让） |
| MPC `max_vel_x` | `0.5` | 决定 MPC 实际前瞻距离 ≈ `max_vel_x×(N−1)×dt_ref` ≈ 5.8 m |
| MPC `grid_size_ref / dt_ref` | `30 / 0.4` | 预测时域 T=(30−1)×0.4=11.6 s |

> ⚠️ costmap 的 `observation_sources` 必须嵌套在 `obstacle_layer:` 命名空间下，
> 否则障碍层收不到任何传感器数据、代价地图始终为空（本项目已修复）。

---

## 9. 本项目相对上游的改动清单

**新增文件**

| 文件 | 说明 |
|------|------|
| `pointcloud_converter/src/livox_sim_bridge.cpp` + `launch/livox_sim_bridge.launch` | 仿真激光雷达桥接节点 |
| `kuavo_mapping/config/mid360_sim.yaml` + `launch/build_map_sim.launch` | 仿真 FAST‑LIO 外参 + 仿真建图入口 |
| `kuavo_navigation/launch/kuavo_navigation_sim.launch` | 仿真导航入口 |
| `kuavo_navigation/scripts/global_plan_follower.py` | 纯追踪运动控制 + 反应式重规划 |
| `kuavo_navigation/scripts/sim_auto_initialpose.py` | 由 Gazebo 真值自动设定初始位姿 |

**修改文件**

- `livox_laser_simulation/`：禁用插件侧 TF 重复发布（修复 `livox_link` 双父节点）、MID360 默认配置、8 m 量程
- `pointcloud_converter/CMakeLists.txt`、`package.xml`：新增 `livox_ros_driver2` 依赖与桥接节点编译目标
- `kuavo_navigation/config/`：`costmap_common_params.yaml`、`local/global_costmap_params.yaml`、`mpc_local_planner_params.yaml`（导航调优）

---

## 10. 常见问题（FAQ）

**Q1. Gazebo 启动报找不到 `.csv` / `world` 文件？**
见 [5.2](#52-️必改项修正硬编码绝对路径重要)，修正 `load_kuavo_gazebo_sim.launch` 里的硬编码绝对路径。

**Q2. 代价地图始终为空、机器人不避障？**
检查 `costmap_common_params.yaml` 中 `observation_sources` 是否在 `obstacle_layer:` 命名空间下；
确认 `/livox/cloud` 有数据（`rostopic hz`）。

**Q3. RViz 中点云/地图整体偏移 1.5 m？**
这是雷达启动高度处理方式（`view_mode`）导致：`visual` 模式 Fixed Frame 设为 `odom`，
`geometry` 模式设为 `map`。建图与导航需使用同一 `view_mode`。

**Q4. FAST‑LIO 报 `no imu`？**
确认 `/livox/imu` 在发布、且 `livox_sim_bridge` 正常运行；检查 `use_sim_time=true` 与 `/clock`。

**Q5. 能直接把仿真雷达参数用到实体机器人吗？**
不能直接照搬。桥接节点（真机改用 `livox_ros_driver2` 官方驱动）、`correction_rpy` 俯仰补偿、
地面过滤、FAST‑LIO 外参、`use_sim_time` 均需按真机重新标定；
但 costmap 尺寸/膨胀、MPC 参数、纯追踪逻辑、0.018 步态阈值可作为真机调优起点直接迁移。

---

> 致谢：机器人本体、运动控制、模型与步态来自 Leju Robotics 开源仓库
> （Gitee：https://gitee.com/leju-robot/kuavo-ros-opensource.git）。
> 本项目在其基础上完成 MID360 仿真接入、SLAM 建图与自主导航集成。
