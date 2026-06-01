#!/bin/bash
# Kuavo导航建图服务管理器启动脚本
# 使用新的架构：通过服务管理器切换导航和建图模式

echo "Starting Kuavo Navigation Service Manager..."

# 获取当前工作目录（从哪里启动就使用哪个路径）
SCRIPT_DIR="$(pwd)"

# 检查并设置ROS环境变量
if [ -z "$ROS_MASTER_URI" ]; then
    ROS_MASTER_URI="http://kuavo_master:11311"
    echo "ROS_MASTER_URI is empty, using default: $ROS_MASTER_URI"
fi

if [ -z "$ROS_IP" ]; then
    ROS_IP="127.0.0.1"
    echo "ROS_IP is empty, using default: $ROS_IP"
fi

echo "Current ROS_MASTER_URI: $ROS_MASTER_URI"
echo "Current ROS_IP: $ROS_IP"

# 进入工作目录
cd "$SCRIPT_DIR"

# 检查是否需要编译
if [ ! -f "devel/setup.bash" ] || [ "src/kuavo_mapping" -nt "devel/lib/python3/dist-packages/kuavo_mapping" ]; then
    echo "Compiling kuavo_slam_ws package..."

    # 调用编译脚本
    if [ -f "./build_kuavo_slam_ws.sh" ]; then
        ./build_kuavo_slam_ws.sh
    else
        echo "build_kuavo_slam_ws.sh not found, using catkin_make..."
        catkin_make
    fi

    if [ $? -ne 0 ]; then
        echo "Compilation failed!"
        exit 1
    fi

    echo "Compilation successful"
fi

# Source ROS环境
source devel/setup.bash

# 可选：传递启动参数
MAP_NAME=${1:-""}
BASE_LOCAL_PLANNER=${2:-"mpc_local_planner"}
ODOM_SOURCE=${3:-"lidar"}

echo "Launch parameters:"
echo "  MAP_NAME: $MAP_NAME"
echo "  BASE_LOCAL_PLANNER: $BASE_LOCAL_PLANNER"
echo "  ODOM_SOURCE: $ODOM_SOURCE"

# 安装systemd服务（如果还没有安装）
if [ ! -f "/etc/systemd/system/kuavo_navigation_service.service" ]; then
    echo "Installing systemd service..."

    # 创建systemd服务文件
    cat > /tmp/kuavo_navigation_service.service << EOF
[Unit]
Description=Kuavo Navigation Service Manager
After=network.target


[Service]
Type=simple
User=leju_kuavo
WorkingDirectory=${SCRIPT_DIR}
Environment=ROS_MASTER_URI=${ROS_MASTER_URI:-http://kuavo_master:11311}
Environment=ROS_IP=${ROS_IP:-127.0.0.1}
Environment=ROS_PACKAGE_PATH=${SCRIPT_DIR}/src
Environment=PYTHONPATH=${SCRIPT_DIR}/devel/lib/python3/dist-packages
ExecStart=/bin/bash -c '. ${SCRIPT_DIR}/devel/setup.bash && exec roslaunch kuavo_mapping navigation_service.launch'
Restart=always
RestartSec=10

[Install]
WantedBy=multi-user.target
EOF

    # 复制到系统目录
    sudo cp /tmp/kuavo_navigation_service.service /etc/systemd/system/

    # 重新加载systemd
    sudo systemctl daemon-reload

    # 启用服务
    sudo systemctl enable kuavo_navigation_service.service

    echo "Systemd service installed and enabled"

    # 删除旧的服务文件（如果存在）
    if [ -f "/etc/systemd/system/kuavo_navigation_services.service" ]; then
        sudo systemctl disable kuavo_navigation_services.service
        sudo rm -f /etc/systemd/system/kuavo_navigation_services.service
        sudo systemctl daemon-reload
        echo "Removed old service file"
    fi

    # 删除临时文件
    rm -f /tmp/kuavo_navigation_service.service

    echo "Please reboot to enable auto-start, or run: sudo systemctl start kuavo_navigation_service.service"
fi

# 启动导航建图服务管理器（如果是直接运行脚本而不是通过systemd）
if [ -z "$SYSTEMD_EXECUTOR" ]; then
    echo "Starting navigation service manager..."
    exec roslaunch kuavo_mapping navigation_service.launch \
        map:="$MAP_NAME" \
        base_local_planner:="$BASE_LOCAL_PLANNER" \
        odom_source:="$ODOM_SOURCE"
fi