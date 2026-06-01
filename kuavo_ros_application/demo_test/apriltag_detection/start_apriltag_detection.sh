#!/bin/bash

# # 设置环境变量
# export XDG_RUNTIME_DIR="/tmp/runtime-$USER"
# mkdir -p "$XDG_RUNTIME_DIR"
# chmod 700 "$XDG_RUNTIME_DIR"

# # 设置显示变量（如果有界面，则需要设置）
# export DISPLAY=:1


# 首先清理可能存在的ros进程
echo "清理现有ROS进程..."
killall -9 rosmaster
killall -9 roscore
sleep 2


# 复制tags.yaml到apriltag_ros包的配置目录
echo "复制tags.yaml配置文件..."
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
APRILTAG_CONFIG_DIR="../../../kuavo_ros_application/src/ros_vision/detection_apriltag/apriltag_ros/config"

# 使用完整路径复制文件
sudo cp "${SCRIPT_DIR}/tags.yaml" "${APRILTAG_CONFIG_DIR}/tags.yaml"

# 启动launch文件
echo "启动机器人头部launch文件..."
source ../../../kuavo_ros_application/devel/setup.bash
roslaunch dynamic_biped load_robot_head.launch &
LAUNCH_PID=$!

# 添加延时，确保所有节点完全启动
sleep 5

# 启动Python程序
echo "启动get_tag_info.py程序..."
python3 get_tag_info.py &
PYTHON_PID=$!

# 等待用户中止
echo "按Ctrl+C停止..."

# 捕捉Ctrl+C终止信号，确保程序结束时清理所有进程
trap 'cleanup' SIGINT
cleanup() {
    echo "停止所有节点..."
    kill $PYTHON_PID 2>/dev/null
    kill $LAUNCH_PID 2>/dev/null
    killall -9 rosmaster
    killall -9 roscore
    exit 0
}

# 使用wait来保持脚本运行
wait
