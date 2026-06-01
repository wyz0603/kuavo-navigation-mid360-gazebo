# 半身接口测试说明文档

本测试套件用于验证机器人上半身（手臂、灵巧手、头部）的各项功能接口。

## 目录
- [H12 预设动作](#H12-预设动作) 
- [手臂贝塞尔规划](#手臂贝塞尔规划)
- [手臂插值运动](#手臂插值运动)
- [手臂运动到目标点](#手臂运动到目标点)
- [手臂IK_FK测试](#手臂IK_FK测试)
- [头部运动测试](#头部运动测试)
- [灵巧手手势测试](#灵巧手手势测试)

## 使用说明

### 配置
- 需求功能包:
  1. humanoid_controllers
     ```bash
     cd ~/kuavo-ros-control
     catkin build humanoid_controllers
     ```
  
  2. humanoid_plan_arm_trajectory
     ```bash
     cd ~/kuavo-ros-control
     catkin build humanoid_plan_arm_trajectory
     ```
  
  3. automatic_test
     ```bash
     cd ~/kuavo-ros-control
     catkin build automatic_test
     ```

### 启动测试
- 本测试套件支持仿真测试和实机测试，使用前需要完成机器人的初始化站立：
  1. 仿真测试时：
     ```bash
     source ~/kuavo-ros-control/devel/setup.bash
     roslaunch humanoid_controllers load_kuavo_mujoco_sim.launch
     ```
  
  2. 实机测试时：
     ```bash
     source ~/kuavo-ros-control/devel/setup.bash
     roslaunch humanoid_controllers load_kuavo_real.launch 
     ```

### 使用方法

#### 单一模块启用：
- 参考每个模块中的说明，启动对应的 launch 文件即可。

#### 启用套件测试：
1. 启动测试套件程序：
   ```bash
   source ~/kuavo-ros-control/devel/setup.bash
   roslaunch automatic_test test_half_body.launch
   ```

2. 根据打印提示选择测试对应的模块或者集成测试。

## 套件模块说明

### H12-预设动作
- launch file: **half_body_H12_action.launch**
- 该模块会查找 **/home/lab/.config/lejuconfig/action_files/** 目录下的 ".tact" 文件进行动作播放，以确保 H12 遥控器能够正确地触发对应动作。
- 单独启用方法：
  ```bash
  roslaunch automatic_test half_body_H12_action.launch
  ```
### 手臂贝塞尔规划
- launch file: **half_body_arm_plan_traj_bezier.launch**
- 该模块会模拟调用贝塞尔插值功能，执行预设动作。
- 该模块需要功能包:**humanoid_plan_arm_trajectory**，请提前编译：
  ```bash
  cd ~/kuavo-ros-control/
  catkin build humanoid_plan_arm_trajectory
  ```
- 单独启用方法：
  ```bash
  roslaunch automatic_test half_body_arm_plan_traj_bezier.launch
  ```

### 手臂插值运动
- launch file: **half_body_arm_traj_control.launch**
- 该模块会分90次插值，将手臂移动到目标位置。将插值过程体现在实际运动上。
- 单独启用方法：
  ```bash
  roslaunch automatic_test half_body_arm_traj_control.launch
  ```

### 手臂运动到目标点
- launch file: **half_body_arm_target_pose.launch**
- 该模块会采用下发目标点位置的方式，控制手臂运动，该功能包会读取[actions](./actions/poses_0.csv) 的位置和时间，进行运动，用户可自定义位置进行测试。
- 单独启用方法：
  ```bash
  roslaunch automatic_test half_body_arm_target_pose.launch
  ```

### 手臂IK_FK测试
- launch file: **half_body_arm_fk_ik.launch**
- 该模块会用末端姿态去调用正解服务（'/ik/fk_srv'），并且根据结果运动到目标姿态。
- 运动到目标姿态后，将正解的结果作为参数调用逆解服务（'/ik/two_arm_hand_pose_cmd_srv'），并且根据结果运动到目标姿态。
- 单独启用方法：
  ```bash
  roslaunch automatic_test half_body_arm_fk_ik.launch
  ```

### 头部运动测试
- launch file: **half_body_head_move.launch**
- 该模块会调用头部运动接口，控制头部的朝向。
- 单独启用方法：
  ```bash
  roslaunch automatic_test half_body_head_move.launch
  ```

### 灵巧手手势测试
- launch file: **half_body_ex_hand_gesture.launch**
- 该模块会调用手势接口，总共 20 种手势，依次执行。
- **该模块执行时，实机执行需要安装灵巧手，且 ROBOT_VERSION 大于等于45**
- **该模块执行时，仿真执行需要指定版本号，且 ROBOT_VERSION=48**
- 单独启用方法：
  ```bash
  roslaunch automatic_test half_body_ex_hand_gesture.launch
  ```

## 测试结果说明
- 本套件旨在测试半身 API 接口是否正常，可直接观察运行测试后机器人是否正常执行动作以及动作是否达标判断。