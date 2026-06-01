# 自动测试包 (Automatic Test Package)

这是一个用于机器人控制的 ROS 自动测试包。它支持在仿真和实际硬件环境下进行测试。

## 概述

automatic_test 包旨在对机器人系统进行自动化测试。目前支持以下测试类型：
- 机器人行走测试
- 单步控制测试（精确步态控制）

## 配置

### 安装依赖

```bash
pip install --upgrade pytest
```

### 编译

```bash
cd <kuavo-ros-control>
catkin build automatic_test
```

### 配置环境变量

打开 `src/automatic_test/automatic_test/env/automatic_test.env` 文件，配置自动化测试的 webhook 地址和机器人序列号。

```bash
AUTOMATIC_TEST_WEBHOOK_URL=https://qyapi.weixin.qq.com/cgi-bin/webhook/send?key=595f7584-001a-42c1-91fc-5c3e6815443d
ROBOT_SERIAL_NUMBER=<机器人编号>
```

### 配置说明

参数文件位于 `config/params.yaml`，包含以下主要配置项：

#### 路径跟踪节点配置 (mpc_path_tracer_node)

1. 运动控制参数
   - `dt`: 控制周期 (0.1 秒)
   - `v_max_linear`: 默认最大线速度 (0.2 m/s)
   - `v_max_angular`: 最大角速度 (0.4 rad/s)

2. 不同路径类型的最大线速度配置
   - `path_velocities`: 针对不同路径类型设置的最大线速度
     - `line`: 直线路径最大速度 (0.6 m/s)
     - `circle`: 圆形路径最大速度 (0.4 m/s)
     - `square`: 方形路径最大速度 (0.4 m/s)
     - `triangle`: 三角形路径最大速度 (0.4 m/s)
     - `scurve`: S形路径最大速度 (0.4 m/s)

3. 测试路径参数
   - 圆形路径 (circle)
     - `radius`: 圆形路径半径 (2.0 米)
   
   - 方形路径 (square)
     - `side_length`: 边长 (2.0 米)
     - `yaw_offset`: 偏航角偏移 (-45.0 度)
   
   - S形路径 (scurve)
     - `length`: 路径长度 (1.5 米)
     - `amplitude`: 波动幅度 (0.5 米)
   
   - 直线路径 (line)
     - `length`: 路径长度 (2.0 米)
     - `yaw_offset`: 偏航角偏移 (0.0 度)
   
   - 三角形路径 (triangle)
     - `length`: 边长 (1.0 米)
     - `yaw_offset`: 偏航角偏移 (-30.0 度)

#### 行走测试节点配置 (test_robot_walk)

- `round`: 测试轮数 (默认: 1)
- `path_types`: 测试路径类型列表 (默认: [triangle, line, circle, square, scurve])

#### 单步控制测试配置 (test_single_step_control)

- `round`: 测试轮数 (默认: 10)
- `forward_step_size`: 前进步态的步长 (默认: 0.08 米)
- `backward_step_size`: 后退步态的步长 (默认: -0.08 米)
- `left_move_step_size`: 左移步态的步长 (默认: 0.05 米)
- `right_move_step_size`: 右移步态的步长 (默认: -0.05 米)
- `rotate_left_step_size`: 左转步态的步长 (默认: 45 度)
- `rotate_right_step_size`: 右转步态的步长 (默认: -45 度)

> 注意：这些参数可以根据实际测试需求进行调整。在修改参数时，请确保值在合理范围内，以确保机器人的安全运行。

## 启动测试

自动化测试提供了多种启动方式，可以根据需要选择不同的测试类型：

### 1. 机器人行走测试

`cmd_vel` 模式(速度控制)

```bash
roslaunch automatic_test test_robot_walk.launch sim:=false test_cmd_vel:=true
```

`cmd_pose` 模式(位置控制)

```bash
roslaunch automatic_test test_robot_walk.launch sim:=false test_cmd_pose:=true
```

`cmd_pose_world` 模式(世界坐标系位置控制)

```bash
roslaunch automatic_test test_robot_walk.launch sim:=false test_cmd_pose_world:=true
```

这个测试会执行预定义的路径（圆形、方形、S形, 三角形, 直线路径），测试机器人的行走能力。

### 2. 单步控制测试

```bash
roslaunch automatic_test test_single_step_control.launch sim:=false
```

这个测试会执行一系列精确的单步动作，包括前进、后退、左右移动和转向，并评估每个步态的执行精度。

### 3. 完整测试套件

```bash
roslaunch automatic_test automatic_test.launch sim:=false
```

这将运行所有可用的测试用例。

### 启动参数说明

- `sim` (默认值: true)：启用/禁用仿真模式
- `log_level` (默认值: INFO)：日志级别设置

## 测试流程

1. 将动捕球安装固定在测试机器人和天轨小车上

2. 确保实验室中的动捕系统已经正常工作, 能够正常获取机器人和天轨小车的 POSE 数据

3. 配置监听动捕数据流的 PC 与测试机器人的 ROS 局域网通信：

```bash
例如(请根据实际情况修改 IP 地址)：
监听动捕数据流的 PC 的 IP：192.168.1.100
测试机器人的 IP：192.168.1.101

### 在监听动捕数据流的 PC 上执行：
echo "export ROS_MASTER_URI=http://192.168.1.101:11311" | sudo tee -a /root/.bashrc
echo "export ROS_IP=192.168.1.100" | sudo tee -a /root/.bashrc

### 在测试机器人上执行：
echo "export ROS_MASTER_URI=http://192.168.1.101:11311" | sudo tee -a /root/.bashrc
echo "export ROS_IP=192.168.1.101" | sudo tee -a /root/.bashrc
```

3. 在**测试机器人**上启动 roscore

```bash
sudo su
roscore
```

4. 启动监听动捕数据流的程序，参阅文档：[动捕数据流监听程序](../motioncapture/readme.md)

5. 启动天轨小车控制程序，并且进入动捕自动跟踪模式, 参阅文档：[天轨小车控制程序](../kuavo_skyrail/README.md)

6. 将机器人挂在天轨小车上，用力移动机器人，测试天轨小车能否正常跟踪机器人

7. **开始测试:**

    7.1  请将机器人移动至测试起点，并保持静止

    7.2  请将机器人放下，直到双足触碰地面
    
    7.3  请将机器人的朝向调整至面朝前方测试空地
    
    7.4  启动测试程序：

```bash
cd <kuavo-ros-control>
sudo su
source devel/setup.bash
roslaunch automatic_test automatic_test.launch sim:=false
```

8. 测试结束后：
   - 对于行走测试：机器人会自动回到测试起点
   - 对于单步控制测试：系统会输出每个步态的执行精度评估结果
   - 所有测试结果都会发送到企业微信通知群

## 测试结果说明

### 行走测试结果
- 路径跟踪完成情况
- 执行轮数统计
- 轨迹跟踪性能评估：
  - 总跟踪时间：从开始跟踪到完成的总时间
  - 总行驶距离：机器人实际行走的总距离
  - 平均速度：总距离除以总时间
  - 距离误差统计：
    - 平均误差：所有采样点的平均距离误差
    - 最大误差：所有采样点中的最大距离误差
    - 最小误差：所有采样点中的最小距离误差
    - RMSE（均方根误差）：反映整体误差大小，对较大误差更敏感
    - 标准差：反映误差的离散程度，值越小表示跟踪越稳定

### 单步控制测试结果
- 每个步态的位置误差（米）
- 每个步态的偏航角误差（度）
- 多轮测试的平均误差统计
