#!/bin/bash

# 添加log函数定义
log() {
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] $*"
}

# # 设置环境变量
# 设置显示变量（如果有界面，则需要设置）
export XDG_RUNTIME_DIR="/tmp/runtime-$USER"
mkdir -p "$XDG_RUNTIME_DIR"
chmod 700 "$XDG_RUNTIME_DIR"
export DISPLAY=:1

# 获取脚本所在目录的绝对路径
KUAVO_WS="../../../kuavo_ros_application"

# 检查工作空间是否存在
if [ ! -d "$KUAVO_WS" ]; then
    log "错误: 未找到kuavo_ros_application工作空间"
    exit 1
fi

# 检查YOLO模型文件是否存在
YOLO_MODEL_PATH="${KUAVO_WS}/src/ros_vision/detection_industrial_yolo/yolo_box_object_detection/scripts/models"
if [ ! -d "$YOLO_MODEL_PATH" ] || [ -z "$(ls -A $YOLO_MODEL_PATH/*.pt 2>/dev/null)" ]; then
    log "错误: YOLO模型文件不存在，请确保模型(.pt文件)已放置在正确位置"
    exit 1
fi

# 首先清理可能存在的ros进程
log "清理现有ROS进程..."
killall -9 rosmaster roscore 2>/dev/null
sleep 2



# 启动机器人头部相机
log "启动机器人头部launch文件..."
source ${KUAVO_WS}/devel/setup.bash
roslaunch dynamic_biped load_robot_head.launch &
LAUNCH_PID=$!
sleep 5

# 启动YOLO检测节点
log "启动YOLO箱子检测程序..."
source ${KUAVO_WS}/devel/setup.bash
roslaunch yolo_box_object_detection yolo_segment_detect.launch &
YOLO_PID=$!

# 等待YOLO节点完全启动
sleep 8

# 检查必要的ROS话题是否存在
if ! rostopic list | grep -q "/object_yolo_box_segment_result"; then
    log "错误: YOLO检测话题未正确发布"
    cleanup
    exit 1
fi

if ! rostopic list | grep -q "/object_yolo_box_tf2_torso_result"; then
    log "错误: 坐标转换话题未正确发布"
    cleanup
    exit 1
fi

# 启动位姿检测程序
log "启动位姿检测程序..."
source ${KUAVO_WS}/devel/setup.bash
python3 yolo_detection_average_info.py &
PYTHON_PID=$!

# 捕捉Ctrl+C终止信号，确保程序结束时清理所有进程
trap 'cleanup' SIGINT SIGTERM
cleanup() {
    log "正在停止所有节点..."
    kill $PYTHON_PID 2>/dev/null
    kill $LAUNCH_PID 2>/dev/null
    kill $YOLO_PID 2>/dev/null
    killall -9 rosmaster roscore 2>/dev/null
    log "清理完成"
    exit 0
}

# 显示订阅的话题信息
log "系统已启动完成。正在监听以下话题:"
log "- /object_yolo_box_segment_result     (相机坐标系下的箱子位置)"
log "- /object_yolo_box_segment_image      (识别结果可视化)"
log "- /object_yolo_box_tf2_torso_result   (机器人基坐标系下的箱子位置)"
log "按Ctrl+C停止..."

# 使用wait来保持脚本运行
wait
