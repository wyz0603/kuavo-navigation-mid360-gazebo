#!/bin/bash
# -*- coding: utf-8 -*
# 检查是否提供了路径参数
if [ $# -ne 1 ]; then
    echo "用法: $0 <路径>"
    exit 1
fi

# 获取输入路径
INPUT_PATH=$1

# 检查路径是否存在
if [ ! -d "$INPUT_PATH" ]; then
    echo "错误: 路径 '$INPUT_PATH' 不存在"
    exit 1
fi

# 寻找_group1.bag和_group2.bag文件
GROUP1_BAG=$(find "$INPUT_PATH" -name "*_group1.bag" | head -n 1)
GROUP2_BAG=$(find "$INPUT_PATH" -name "*_group2.bag" | head -n 1)

# 检查是否找到文件
if [ -z "$GROUP1_BAG" ]; then
    echo "错误: 在 '$INPUT_PATH' 中未找到 _group1.bag 文件"
    exit 1
fi

if [ -z "$GROUP2_BAG" ]; then
    echo "错误: 在 '$INPUT_PATH' 中未找到 _group2.bag 文件"
    exit 1
fi

echo "找到 group1 bag: $GROUP1_BAG"
echo "找到 group2 bag: $GROUP2_BAG"


# 调用rosservice来改变控制模式
echo "正在调用服务 /arm_traj_change_mode 设置控制模式为1..."
rosservice call /arm_traj_change_mode "control_mode: 1" > /dev/null 2>&1

sleep 1
rosservice call /arm_traj_change_mode "control_mode: 2" > /dev/null 2>&1


# 调用VR_test.py并加载_group1.bag文件
echo "正在运行 VR_test.py..."
python3 $(rospack find automatic_test)/scripts/automatic_VR_test/VR_test.py "$GROUP1_BAG" &

# 延迟3秒等待启动完成
echo "等待3秒..."
sleep 3

# 调用roslaunch启动rosbag_mock_VR.launch
echo "正在启动 rosbag_mock_VR.launch..."
roslaunch automatic_test rosbag_mock_VR.launch rosbag_path:=$GROUP2_BAG > /dev/null 2>&1 &
LAUNCH_PID=$!

# 等待bag播放完成
echo "等待bag播放完成..."
rosbag info $GROUP2_BAG | grep "duration"
DURATION=$(rosbag info $GROUP2_BAG | grep "duration" | awk '{print $2}' | sed 's/s//')
# 增加5秒缓冲时间
SLEEP_TIME=$(echo "$DURATION + 5" | bc)
echo "bag持续时间: $DURATION 秒，等待 $SLEEP_TIME 秒..."
sleep $SLEEP_TIME

# 结束launch进程
echo "bag播放完成，终止launch进程..."
kill $LAUNCH_PID
wait $LAUNCH_PID 2>/dev/null

echo "测试完成"
# 调用rosservice来改变控制模式
echo "正在调用服务 /arm_traj_change_mode 设置控制模式为1..."
rosservice call /arm_traj_change_mode "control_mode: 1" > /dev/null 2>&1

sleep 1
rosservice call /arm_traj_change_mode "control_mode: 2" > /dev/null 2>&1
