#!/bin/bash

# 雷达检测工具启动脚本
# 用于检测雷达点云频率和点云总数

set -e

# 获取当前脚本的绝对路径
CURRENT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 获取application路径
PROJECT_DIR="$(dirname "$(dirname "$(dirname "$CURRENT_DIR")")")"
echo "kuavo_ros_application path: $PROJECT_DIR"

source /opt/ros/noetic/setup.bash

cd $PROJECT_DIR
echo -e "${YELLOW}⚠️  正在清理旧的构建文件...${NC}"
rm -rf ./src/kuavo_slam_ws/build
rm -rf ./src/kuavo_slam_ws/devel
rm -rf ./.catkin_tools
echo -e "${GREEN}✅ 清理完成${NC}"

echo -e "${BLUE}🔨 正在编译Livox雷达驱动...${NC}"
./src/kuavo_slam_ws/src/livox_ros_driver2/build.sh ROS1
echo -e "${GREEN}✅ 编译完成${NC}"

source ./src/kuavo_slam_ws/devel/setup.bash

echo -e "${BLUE}🚀 正在启动Livox Mid360雷达节点...${NC}"
roslaunch livox_ros_driver2 start_mid360.launch &
echo -e "${GREEN}✅ 雷达节点已启动（后台运行）${NC}"
echo -e "${YELLOW}💡 提示: 雷达节点正在后台运行，请等待几秒钟让雷达完全启动${NC}"

cd $CURRENT_DIR


# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 脚本路径
TOOL_SCRIPT=$PROJECT_DIR/tools/checktools/lidar_tools/lidar_test_tool.py


# 默认参数
DURATION=30
MIN_FREQUENCY=5.0
MIN_POINT_COUNT=1000
TOPIC="/lidar"

# 显示帮助信息
show_help() {
    echo -e "${BLUE}雷达检测工具启动脚本${NC}"
    echo ""
    echo "用法: $0 [选项]"
    echo ""
    echo "选项:"
    echo "  -d, --duration SECONDS     检测持续时间（秒），默认30秒"
    echo "  -f, --frequency HZ         最小点云频率阈值（Hz），默认5.0"
    echo "  -p, --points COUNT         最小点云总数阈值，默认1000"
    echo "  -t, --topic TOPIC          点云话题名称，默认/lidar"
    echo "  -h, --help                 显示此帮助信息"
    echo ""
    echo "示例:"
    echo "  $0                         使用默认参数检测30秒"
    echo "  $0 -d 60 -f 10 -p 2000    检测60秒，频率阈值10Hz，点数阈值2000"
    echo "  $0 --topic /rslidar_points 使用自定义话题"
    echo ""
}

# 解析命令行参数
while [[ $# -gt 0 ]]; do
    case $1 in
        -d|--duration)
            DURATION="$2"
            shift 2
            ;;
        -f|--frequency)
            MIN_FREQUENCY="$2"
            shift 2
            ;;
        -p|--points)
            MIN_POINT_COUNT="$2"
            shift 2
            ;;
        -t|--topic)
            TOPIC="$2"
            shift 2
            ;;
        -h|--help)
            show_help
            exit 0
            ;;
        *)
            echo -e "${RED}错误: 未知参数 $1${NC}"
            show_help
            exit 1
            ;;
    esac
done

# 检查Python脚本是否存在
if [[ ! -f "$TOOL_SCRIPT" ]]; then
    echo -e "${RED}错误: 找不到雷达检测工具脚本: $TOOL_SCRIPT${NC}"
    exit 1
fi

# 检查脚本权限
if [[ ! -x "$TOOL_SCRIPT" ]]; then
    echo -e "${YELLOW}设置脚本执行权限...${NC}"
    chmod +x "$TOOL_SCRIPT"
fi

# 检查ROS环境
if [[ -z "$ROS_DISTRO" ]]; then
    echo -e "${YELLOW}警告: 未检测到ROS环境，尝试source ROS环境...${NC}"
    
    # 尝试source常见的ROS环境
    if [[ -f "/opt/ros/noetic/setup.bash" ]]; then
        source /opt/ros/noetic/setup.bash
        echo -e "${GREEN}已加载ROS Noetic环境${NC}"
    elif [[ -f "/opt/ros/melodic/setup.bash" ]]; then
        source /opt/ros/melodic/setup.bash
        echo -e "${GREEN}已加载ROS Melodic环境${NC}"
    else
        echo -e "${RED}错误: 无法找到ROS环境，请手动source ROS环境后重试${NC}"
        echo "例如: source /opt/ros/noetic/setup.bash"
        exit 1
    fi
fi

# 检查工作空间
if [[ -f "$SCRIPT_DIR/../../../devel/setup.bash" ]]; then
    echo -e "${YELLOW}检测到ROS工作空间，正在加载...${NC}"
    source "$SCRIPT_DIR/../../../devel/setup.bash"
    echo -e "${GREEN}已加载ROS工作空间环境${NC}"
fi

# 检查依赖包
echo -e "${BLUE}检查依赖包...${NC}"
python3 -c "import rospy, numpy" 2>/dev/null || {
    echo -e "${YELLOW}正在安装依赖包...${NC}"
    pip3 install numpy --user
    echo -e "${GREEN}依赖包安装完成${NC}"
}

# 显示检测参数
echo -e "${BLUE}检测参数:${NC}"
echo "  - 检测时长: ${DURATION} 秒"
echo "  - 最小频率阈值: ${MIN_FREQUENCY} Hz"
echo "  - 最小点云总数阈值: ${MIN_POINT_COUNT}"
echo "  - 订阅话题: ${TOPIC}"
echo ""

# 启动检测
echo -e "${GREEN}启动雷达检测工具...${NC}"
echo -e "${YELLOW}提示: 按 Ctrl+C 可以中断检测${NC}"
echo ""

cd "$SCRIPT_DIR"
python3 "$TOOL_SCRIPT" \
    --duration "$DURATION" \
    --min-frequency "$MIN_FREQUENCY" \
    --min-point-count "$MIN_POINT_COUNT" \
    --topic "$TOPIC"

echo ""
echo -e "${GREEN}雷达检测完成！${NC}"