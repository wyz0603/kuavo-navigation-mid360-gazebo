if [ -f "CATKIN_IGNORE" ]; then
    echo "检测到CATKIN_IGNORE文件，正在删除..."
    rm CATKIN_IGNORE
fi

if [ -f /etc/os-release ] && grep -q 'Ubuntu 20.04' /etc/os-release; then
    echo "检测到当前系统为Ubuntu 20.04，直接运行build_kuavo_slam_ws.sh脚本..."
    bash build_kuavo_slam_ws.sh
else
    echo "当前系统不是Ubuntu 20.04，进入docker容器并运行build_kuavo_slam_ws.sh脚本..."
    docker run --rm -v "$(pwd)":/workspace -w /workspace kuavo_navigation bash build_kuavo_slam_ws.sh
fi
