# How-to Use Kuavo Teach Pendant

## 介绍

本工具提供在 0-torque 模式下,可手动拖摆动机器人手臂, 并录制摆动时的手臂关节数据, 并提供重播录制动作的功能.

## 启动机器人程序

首先需要进行手臂零点标定, 如果您的机械手臂是多圈的且您之前已经标定, 请忽略该步骤, 否则请摆正手臂后, 执行以下命令进行标定:
```bash
source devel/setup.bash 
rosrun  hardware_node setZero.sh
```
然后启动机器人:
```bash
source devel/setup.bash
roslaunch humanoid_controllers load_kuavo_real.launch teach_pendant:=1 # 正常具有全身的机器人

# 只有上半身的机器人(轮臂机器人), 请运行以下命令:
source devel/setup.bash
roslaunch humanoid_controllers load_kuavo_real_half_up_body.launch teach_pendant:=1 # 只有上半身的机器人
```

启动之后需要按 o 键使能电机

## 录制数据包

### 录制rosbag话题数据
请先确保在 0-torque 模式下启动机器人, 然后执行以下命令录制话题数据:
```bash
source devel/setup.bash
roslaunch teach_pendant launch_teach_pendant_record_rosbag.launch save_bag:=/path/to/save/bagname
```
其中 `/path/to/save/bagname` 为保存的rosbag文件完整路径名


### 也可以录制关键帧数据，通过按下键盘‘s’记录一帧关键帧动作

```bash
source devel/setup.bash
roslaunch teach_pendant launch_teach_pendant_record_file.launch save_file:=/path/to/save/filename
```

其中 `/path/to/save/filename` 为保存的记录关键帧文件完整路径名


以下是记录关键帧时终端输出的信息:

```
Recording started.

Key commands:
  s - Save the current joint state to file
  Ctrl+C - Exit the program
```
此时按下‘s’会记录一帧数据

记录了两个关键帧的文件实例如下(单位是角度):
```bash
frame_1: [6.688944668069868, 0.26222175954934457, 0.8742850281282628, 0.043699139520925044, 0.1748552065106258, -0.17486768318044163, 0.04366378011757604, 8.587659186481185, -0.7212213275411588, 1.1147072424638924, -0.6557031046764389, 0.5901067615630409, 0.939812979855269, -0.6774928350594672]
frame_2: [6.674225439283481, 0.2622862835336425, 0.8742997290724711, 0.04366009126140115, 0.1748897410867445, -0.1748426016729588, 0.04367398806815109, -12.56390743084296, -51.71382905849491, -0.3496471881561264, -0.9398426465709862, 32.632482053002796, 6.797494131713388, -12.589631500169236]
```

## 播放录制的动作
请先确保机器人在正常的分支下启动（能全身正常站立）, 然后执行以下命令播放之前录制的动作:

### 播放记录的rosbag文件
```bash
source devel/setup.bash
roslaunch teach_pendant launch_teach_pendant_play_rosbag.launch bag_file:=/path/to/bag
```
`/path/to/bag`即之前录制的话题数据bag文件

### 播放记录的关键帧文件

```bash
source devel/setup.bash
roslaunch teach_pendant launch_teach_pendant_play_file.launch file:=/path/to/file interval_sec:=1
```
`/path/to/file`即之前录制的话题数据bag文件
`interval_sec`为两个关键帧之间的执行时间间隔，该参数可以控制动作执行速度，单位为秒，最小值为0.5秒