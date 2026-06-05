# Kuavo Humanoid — MID360 LiDAR Simulation & Autonomous Navigation

**语言 / Language：[中文](./README.md) | English**

> Integrates a Livox MID360 LiDAR into the Gazebo simulation of the Leju Kuavo
> humanoid robot, wiring up the full pipeline of
> **simulated LiDAR → FAST‑LIO odometry/SLAM mapping → move_base autonomous navigation**,
> together with navigation tuning and a Pure‑Pursuit motion controller.

---

## Table of Contents

- [1. Overview](#1-overview)
- [2. Repositories & Upstream URLs](#2-repositories--upstream-urls)
- [3. Architecture & Data Flow](#3-architecture--data-flow)
- [4. Prerequisites](#4-prerequisites)
- [5. Deployment & Build](#5-deployment--build)
- [6. Running](#6-running)
- [7. Testing & Verification](#7-testing--verification)
- [8. Key Parameters](#8-key-parameters)
- [9. Changes Relative to Upstream](#9-changes-relative-to-upstream)
- [10. FAQ](#10-faq)

---

## 1. Overview

The project consists of two cooperating ROS (Noetic) workspaces:

| Project | Role |
|---------|------|
| **kuavo-ros-opensource** | Robot body + Gazebo simulation (MPC‑WBC control, 8‑phase gait, Livox MID360 sim plugin; publishes `/scan`, `/livox/imu`) |
| **kuavo_ros_application** | SLAM + navigation stack (FAST‑LIO, octomap, move_base, MPC local planner) — **this repo** |

Core features:

- **Simulated LiDAR bridge** — the C++ node `livox_sim_bridge` converts the
  legacy `sensor_msgs/PointCloud` (`/scan`) emitted by the Gazebo plugin into the
  `livox_ros_driver2/CustomMsg` required by FAST‑LIO and the `PointCloud2`
  required by navigation, applying mounting‑pose correction to the cloud/IMU.
- **SLAM mapping** — FAST‑LIO publishes `/Odometry`; octomap_server builds a 2D
  occupancy grid.
- **Autonomous navigation** — move_base (global Dijkstra/A* + MPC local planner)
  plus the Pure‑Pursuit node `global_plan_follower` (owns `/cmd_vel`, triggers
  costmap clear + replan when blocked).
- **Gait coupling** — the `/cmd_vel` speed threshold (0.018 m/s) drives the
  robot's stance ↔ walk gait transition.

---

## 2. Repositories & Upstream URLs

This repo is forked / migrated from Leju Robotics' official repositories:

| Project | Upstream (official) |
|---------|---------------------|
| **kuavo-ros-opensource** (robot body + Gazebo sim) | **Gitee: https://gitee.com/leju-robot/kuavo-ros-opensource.git** |
| **kuavo_ros_application** (SLAM + navigation) | LejuHub: https://www.lejuhub.com/ros-application-team/kuavo_ros_application.git |

> 📌 **Original Gitee repository**: `https://gitee.com/leju-robot/kuavo-ros-opensource.git` (master branch).
> This project only adds MID360 simulation integration and the navigation stack on
> top of it; body control, models and gait remain © Leju Robotics.

---

## 3. Architecture & Data Flow

```
┌─────────────────────────── kuavo-ros-opensource (Gazebo) ───────────────────────────┐
│  Livox MID360 sim plugin               IMU plugin                                     │
│        │ /scan (PointCloud, ~10Hz)         │ /livox/imu (Imu, ~1000Hz)               │
└────────┼───────────────────────────────────┼─────────────────────────────────────────┘
         │                                   │
┌────────▼───────────────────────────────────▼──── kuavo_ros_application ──────────────┐
│  livox_sim_bridge (added by this project)                                             │
│     /scan ──► /livox/lidar (CustomMsg)   +  IMU correction ──► /livox/imu_corrected   │
│                         │                                                              │
│                  ┌──────▼──────┐                                                       │
│                  │  FAST-LIO   │ ──► /Odometry  +  /livox/cloud (PointCloud2)          │
│                  └──────┬──────┘            │                                          │
│        ┌────────────────┴────────┐          │                                          │
│   octomap_server            move_base (global planner + MPC local planner)             │
│   2D occupancy grid              │  global_costmap / local_costmap                     │
│                                  ▼                                                      │
│                       global_plan_follower (Pure Pursuit) ──► /cmd_vel ──► robot gait  │
└───────────────────────────────────────────────────────────────────────────────────────┘
```

---

## 4. Prerequisites

- Ubuntu 20.04
- ROS Noetic (`/opt/ros/noetic`)
- Gazebo (installed with ROS Noetic)
- `python3-catkin-tools` (`catkin build`)
- Robot version environment variable `ROBOT_VERSION` (e.g. `45` = version 4.5)
- Dependencies (installed automatically on first build via rosdep / apt):
  `octovis`, `ros-noetic-tf2-sensor-msgs`, `libyaml-cpp-dev`, `libpcap-dev`, etc.

```bash
# Set the robot version (match your robot model; 45 = v4.5)
echo 'export ROBOT_VERSION=45' >> ~/.bashrc
source ~/.bashrc
```

---

## 5. Deployment & Build

### 5.1 Clone this repository

This is a **monorepo** — both workspaces are bundled as subdirectories:

```bash
git clone https://github.com/wuyangzhi03-dev/kuavo-navigation-mid360-gazebo.git
cd kuavo-navigation-mid360-gazebo
# Subdirectories:
#   kuavo-ros-opensource/    robot body + Gazebo simulation
#   kuavo_ros_application/   SLAM + navigation

# The commands below use ~/kuavo-ros-opensource and ~/kuavo_ros_application as
# shorthand; symlink them to your home so those commands work as-is:
ln -s "$(pwd)/kuavo-ros-opensource"  ~/kuavo-ros-opensource
ln -s "$(pwd)/kuavo_ros_application" ~/kuavo_ros_application
```

> The robot-body subdirectory originates from Leju's official Gitee repository
> `https://gitee.com/leju-robot/kuavo-ros-opensource.git` (see [Section 2](#2-repositories--upstream-urls)).

### 5.2 ⚠️ Required edit: fix hard‑coded absolute paths (important)

The simulation launch file of `kuavo-ros-opensource`,
`src/humanoid-control/humanoid_controllers/launch/load_kuavo_gazebo_sim.launch`,
contains **hard‑coded absolute paths** pointing at the original dev machine. After
cloning you **must** change them to your own paths, otherwise Gazebo fails to load
the LiDAR / world:

```xml
<!-- Point to mid360.csv under your own kuavo_ros_application clone -->
<arg name="livox_csv_file"
     default="$HOME/kuavo_ros_application/src/kuavo_slam_ws/src/livox_laser_simulation/scan_mode/mid360.csv"/>
<!-- Point to your own world file -->
<arg name="world_name" default="$HOME/world.world"/>
```

Also confirm the following (defaults are already correct for simulation):

```xml
<arg name="use_livox"   default="true"/>   <!-- load MID360 in Gazebo -->
<arg name="livox_topic" default="/scan"/>  <!-- must match livox_sim_bridge input -->
```

### 5.3 Build the robot body (kuavo-ros-opensource)

```bash
cd ~/kuavo-ros-opensource
sudo su                       # this workspace requires a root environment
source /opt/ros/noetic/setup.bash
catkin build                  # first build compiles the cppad model (~minutes)
exit
```

### 5.4 Build SLAM/navigation (kuavo_ros_application)

`kuavo_slam_ws` is a **standalone catkin workspace**. Use the bundled script for a
one‑shot build (it builds Livox‑SDK, `livox_ros_driver2` (ROS1), FAST‑LIO, octomap,
and this project's bridge node, etc.):

```bash
cd ~/kuavo_ros_application/src/kuavo_slam_ws
source /opt/ros/noetic/setup.bash
./build_kuavo_slam_ws.sh
```

> Rebuild only the two packages changed by this project (for verification):
> ```bash
> cd ~/kuavo_ros_application/src/kuavo_slam_ws
> catkin build --workspace "$(pwd)" pointcloud_converter livox_laser_simulation
> ```

---

## 6. Running

> Each terminal must source the right environment first. Convention:
> ```bash
> # robot body environment
> source ~/kuavo-ros-opensource/devel/setup.bash
> # overlay the navigation environment
> source ~/kuavo_ros_application/src/kuavo_slam_ws/devel/setup.bash --extend
> ```

### 6.1 Launch the Gazebo simulation (terminal 1)

```bash
sudo su
source ~/kuavo-ros-opensource/devel/setup.bash
roslaunch humanoid_controllers load_kuavo_gazebo_sim.launch \
    use_livox:=true joystick_type:=bt2pro rviz:=false
```

The robot stands up in Gazebo and starts publishing `/scan` and `/livox/imu`.

### 6.2 Mapping (terminal 2)

```bash
source ~/kuavo-ros-opensource/devel/setup.bash
source ~/kuavo_ros_application/src/kuavo_slam_ws/devel/setup.bash --extend
roslaunch kuavo_mapping build_map_sim.launch
```

Tele‑operate the robot around the scene with the joystick to accumulate the map.
This brings up `livox_sim_bridge` + FAST‑LIO + octomap.

**Save the map** (output to `~/maps/<name>.{pcd,pgm,yaml}`):

```bash
# Save via the project's existing mapping service (navigation_service must be running)
rosservice call /save_map_service "map_name: 'sim_map_1'"
```

> Note: `build_map_sim.launch` sets `pcd_save_en=true`, so FAST‑LIO also dumps the
> cloud on Ctrl‑C as a fallback.

### 6.3 Autonomous navigation (terminal 2, stop mapping first)

```bash
source ~/kuavo-ros-opensource/devel/setup.bash
source ~/kuavo_ros_application/src/kuavo_slam_ws/devel/setup.bash --extend
roslaunch kuavo_navigation kuavo_navigation_sim.launch map:=sim_map_1
```

In RViz, publish a goal with **2D Nav Goal**; the robot follows the global path,
avoiding obstacles, to reach the goal.

Common optional arguments:

```bash
# Switch local planner (default: mpc_local_planner)
roslaunch kuavo_navigation kuavo_navigation_sim.launch map:=sim_map_1 base_local_planner:=dwa_local_planner
# Disable Pure Pursuit; let MPC drive /cmd_vel directly
roslaunch kuavo_navigation kuavo_navigation_sim.launch map:=sim_map_1 use_global_plan_follower:=false
# Record a navigation data bag
roslaunch kuavo_navigation kuavo_navigation_sim.launch map:=sim_map_1 record_nav_data:=true
```

---

## 7. Testing & Verification

### 7.1 Topic connectivity

```bash
# Simulation side
rostopic hz /scan          # ≈10 Hz  PointCloud
rostopic hz /livox/imu     # ≈1000 Hz Imu

# Bridge + SLAM side
rostopic hz /livox/lidar   # ≈10 Hz  CustomMsg
rostopic hz /livox/cloud   # ≈10 Hz  PointCloud2
rostopic hz /Odometry      # FAST-LIO odometry (continuous, no gaps)
rostopic echo -n1 /projected_map   # octomap 2D occupancy grid
```

### 7.2 TF check

```bash
rosrun rqt_tf_tree rqt_tf_tree
# Confirm livox_link has exactly one parent (provided by robot_state_publisher via the URDF chain)
```

### 7.3 Navigation closed‑loop verification

- After sending a 2D Nav Goal in RViz, `/cmd_vel` keeps publishing (`rostopic echo /cmd_vel`).
- The robot walks toward the goal in Gazebo, avoiding obstacles; when fully blocked
  it automatically runs `clear_costmaps` and replans.
- Once inside goal tolerance (xy 0.25 m / yaw 0.20 rad) it stops and the gait
  returns to stance.

**Success criteria**: no `no imu/lidar` warning from FAST‑LIO; the octomap grid
accumulates as the robot moves; during navigation `/cmd_vel` keeps publishing and
drives the robot to the goal.

---

## 8. Key Parameters

Core tunables of the simulation pipeline (in the corresponding launch / config):

| Parameter | Default | Purpose |
|-----------|---------|---------|
| `correction_rpy` | `[3.14159, 0.11, 0.0]` | MID360 inverted mount + pitch compensation; rotates LiDAR/IMU to an upright reference |
| `cloud_z_offset` | `1.5` (visual mode) | Constant Z added to each cloud point so the floor lands on the grid |
| `lidar_min_z` | `-1.2` | Filters the ground "halo" noise under the robot's feet |
| `0.018 m/s` | — | `/cmd_vel` linear‑speed threshold for stance ↔ walk gait switching |
| local_costmap `width/height` | `6.0 m` | Local costmap size (upstream 3.0 → 6.0 for enough MPC reaction distance) |
| `inflation_radius` | `0.5 m` | Inflation radius (upstream 1.2 → 0.5 to avoid saturating narrow passages) |
| `cost_scaling_factor` | `3` | Cost decay slope (upstream 8 → 3 so MPC can feel the gradient and avoid early) |
| MPC `max_vel_x` | `0.5` | Bounds MPC real lookahead ≈ `max_vel_x×(N−1)×dt_ref` ≈ 5.8 m |
| MPC `grid_size_ref / dt_ref` | `30 / 0.4` | Prediction horizon T=(30−1)×0.4=11.6 s |

> ⚠️ In the costmap, `observation_sources` **must** be nested under the
> `obstacle_layer:` namespace, otherwise the obstacle layer receives no sensor data
> and the costmap stays blank (already fixed in this project).

---

## 9. Changes Relative to Upstream

**New files**

| File | Description |
|------|-------------|
| `pointcloud_converter/src/livox_sim_bridge.cpp` + `launch/livox_sim_bridge.launch` | Simulated LiDAR bridge node |
| `kuavo_mapping/config/mid360_sim.yaml` + `launch/build_map_sim.launch` | Sim FAST‑LIO extrinsics + sim mapping entry |
| `kuavo_navigation/launch/kuavo_navigation_sim.launch` | Sim navigation entry |
| `kuavo_navigation/scripts/global_plan_follower.py` | Pure‑Pursuit motion control + reactive replan |
| `kuavo_navigation/scripts/sim_auto_initialpose.py` | Auto initial pose from Gazebo ground truth |

**Modified files**

- `livox_laser_simulation/`: disable plugin‑side duplicate TF publishing (fixes the
  `livox_link` double‑parent issue), MID360 default config, 8 m range
- `pointcloud_converter/CMakeLists.txt`, `package.xml`: add `livox_ros_driver2`
  dependency and the bridge build target
- `kuavo_navigation/config/`: `costmap_common_params.yaml`,
  `local/global_costmap_params.yaml`, `mpc_local_planner_params.yaml` (navigation tuning)

---

## 10. FAQ

**Q1. Gazebo fails to find the `.csv` / `world` file?**
See [5.2](#52--required-edit-fix-hard-coded-absolute-paths-important); fix the
hard‑coded absolute paths in `load_kuavo_gazebo_sim.launch`.

**Q2. Costmap stays blank and the robot doesn't avoid obstacles?**
Check that `observation_sources` is under the `obstacle_layer:` namespace in
`costmap_common_params.yaml`; confirm `/livox/cloud` has data (`rostopic hz`).

**Q3. The cloud/map is shifted by ~1.5 m in RViz?**
This is the LiDAR start‑height handling (`view_mode`): `visual` mode uses Fixed
Frame `odom`, `geometry` mode uses `map`. Mapping and navigation must use the same
`view_mode`.

**Q4. FAST‑LIO reports `no imu`?**
Confirm `/livox/imu` is publishing and `livox_sim_bridge` is running; check
`use_sim_time=true` and `/clock`.

**Q5. Can the simulation LiDAR parameters be used directly on the real robot?**
Not directly. The bridge node (use the `livox_ros_driver2` official driver on
hardware), `correction_rpy` pitch compensation, ground filtering, FAST‑LIO
extrinsics and `use_sim_time` all need recalibration for the real robot. However,
costmap dimensions/inflation, MPC parameters, the Pure‑Pursuit logic and the 0.018
gait threshold transfer over directly as a starting point for real‑robot tuning.

---

> Acknowledgement: the robot body, motion control, models and gait come from Leju
> Robotics' open‑source repository
> (Gitee: https://gitee.com/leju-robot/kuavo-ros-opensource.git). This project adds
> MID360 simulation integration, SLAM mapping and autonomous navigation on top of it.
