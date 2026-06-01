#!/bin/bash

# 统一相机测试工具启动脚本
# 功能：启动相机驱动并运行综合测试

set -e

# 颜色代码
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 脚本目录
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# 工作空间目录 - 从camera_tools目录向上找到kuavo_ros_application
WORKSPACE_DIR="$(dirname "$(dirname "$(dirname "$SCRIPT_DIR")")")"

echo $WORKSPACE_DIR

HEAD_ORBBEC_CAMERA_EXIST=0
HEAD_RS_CAMERA_EXIST=0
HAS_LEFT_WRIST=0
HAS_RIGHT_WRIST=0

HEAD_CAMERA_TYPE=orbbec_camera

# 日志函数
log() {
    echo -e "${GREEN}[$(date '+%Y-%m-%d %H:%M:%S')]${NC} $1"
}

warn() {
    echo -e "${YELLOW}[$(date '+%Y-%m-%d %H:%M:%S')]${NC} $1"
}

error() {
    echo -e "${RED}[$(date '+%Y-%m-%d %H:%M:%S')]${NC} $1"
}

info() {
    echo -e "${BLUE}[$(date '+%Y-%m-%d %H:%M:%S')]${NC} $1"
}

# 检查并安装Python依赖
check_and_install_dependencies() {

    log "安装依赖..."
    
if [[ $(lsb_release -rs) == "22.04" ]]; then
    echo "Ubuntu 22.04 detected."

    DDYNAMIC_CONFIGURE_PATH=$(rospack find ddynamic_reconfigure 2>/dev/null || echo "")
    if [ -n "$DDYNAMIC_CONFIGURE_PATH" ] && [ "$DDYNAMIC_CONFIGURE_PATH" == "/opt/ros/noetic/share/ddynamic_reconfigure" ]; then
        echo "ddynamic_reconfigure 库已找到，并且路径为 /opt/ros/noetic/share/ddynamic_reconfigure。跳过下载。"
    else
        echo "Installing ddynamic_reconfigure..."
        sudo -i <<EOF
        rm -rf /tmp/third_party
        cd /tmp
        mkdir -p third_party/src
        cd third_party/src
        git clone https://gitee.com/leju-robot/ddynamic_reconfigure
        if [ $? -ne 0 ]; then
            echo "Error: 无法从 gitee 仓库拉取 ddynamic_reconfigure，请重试"
            exit 1
        fi
        cd ..
        source /opt/ros/noetic/setup.bash
        catkin_make install -DCMAKE_INSTALL_PREFIX=/opt/ros/noetic
        rm -rf /tmp/third_party
        exit
EOF
    fi

    POLLED_CAMERA_PATH=$(rospack find polled_camera 2>/dev/null || echo "")
    CAMERA_CALIBRATION_PARSERS_PATH=$(rospack find camera_calibration_parsers 2>/dev/null || echo "")
    CAMERA_INFO_MANAGER_PATH=$(rospack find camera_info_manager 2>/dev/null || echo "")
    IMAGE_TRANSPORT_PATH=$(rospack find image_transport 2>/dev/null || echo "")
    if [ -n "$POLLED_CAMERA_PATH" ] && [ "$POLLED_CAMERA_PATH" == "/opt/ros/noetic/share/polled_camera" ] && \
       [ -n "$CAMERA_CALIBRATION_PARSERS_PATH" ] && [ "$CAMERA_CALIBRATION_PARSERS_PATH" == "/opt/ros/noetic/share/camera_calibration_parsers" ] && \
       [ -n "$CAMERA_INFO_MANAGER_PATH" ] && [ "$CAMERA_INFO_MANAGER_PATH" == "/opt/ros/noetic/share/camera_info_manager" ] && \
       [ -n "$IMAGE_TRANSPORT_PATH" ] && [ "$IMAGE_TRANSPORT_PATH" == "/opt/ros/noetic/share/image_transport" ]; then
        echo "所有图像相关包已找到，并且路径为 /opt/ros/noetic/share/xxx。跳过下载。"
    else
        echo "Installing image_common and related packages..."
        sudo -i <<EOF
        rm -rf /tmp/third_party
        cd /tmp
        mkdir -p third_party/src
        cd third_party/src
        git clone https://gitee.com/leju-robot/image_common --branch=noetic-devel
        if [ $? -ne 0 ]; then
            echo "Error: 无法从 gitee 仓库拉取 image_common，请重试"
            exit 1
        fi
        cd ..
        source /opt/ros/noetic/setup.bash
        catkin_make install -DCMAKE_INSTALL_PREFIX=/opt/ros/noetic
        rm -rf /tmp/third_party
        exit
EOF
    fi
fi


source /opt/ros/noetic/setup.bash
cd $WORKSPACE_DIR
catkin build apriltag_ros
# catkin build 

}

# 编译apriltag相关包
compile_apriltag_packages() {
    log "编译apriltag相关包..."
    cd $WORKSPACE_DIR   
    # 检查工作空间
    if [ ! -f "devel/setup.bash" ]; then
        log "工作空间未编译，开始编译..."
        
        # 编译apriltag相关包
        log "编译apriltag_ros包..."
        catkin build apriltag_ros --workspace . || {
            error "apriltag_ros包编译失败"
            return 1
        }
        
        log "编译dynamic_biped包..."
        catkin build dynamic_biped --workspace . || {
            error "dynamic_biped包编译失败"
            return 1
        }
        
        log "✅ apriltag相关包编译完成"
    else
        log "✅ 工作空间已编译"
    fi
}

# 检查ROS环境
check_ros() {
    if [ -z "$ROS_DISTRO" ]; then
        error "ROS环境未配置，请先source ROS环境"
        echo "例如: source /opt/ros/noetic/setup.bash"
        return 1
    fi
    log "ROS环境: $ROS_DISTRO"
    return 0
}

# 检查工作空间
check_workspace() {
    log "脚本目录: $SCRIPT_DIR"
    log "工作空间目录: $WORKSPACE_DIR"
    
    log "工作空间: $WORKSPACE_DIR"
    cd "$WORKSPACE_DIR"
    
    # 检查是否有build目录或devel目录
    if [ ! -d "build" ] && [ ! -d "devel" ]; then
        log "初始化工作空间..."
        catkin init 2>/dev/null || true
    fi
    
    log "开始编译相机相关包..."
    if catkin build kuavo_camera orbbec_camera; then
        log "✅ 相机包编译成功"
        return 0
    else
        error "❌ 相机包编译失败"
        return 0
    fi
}

# 检查奥比相机是否存在
check_orbbec_camera() {
    log "检查奥比相机是否存在..."
    
    # 使用包里的工具检查
    local list_devices_script="$WORKSPACE_DIR/src/OrbbecSDK_ROS1/scripts/list_ob_devices.sh"
    
    if [ ! -f "$list_devices_script" ]; then
        error "❌ 未找到奥比相机检测脚本: $list_devices_script"
        return 1
    fi
    
    # 给脚本添加执行权限
    chmod +x "$list_devices_script" 2>/dev/null || true
    
    # 运行检测脚本
    local output
    if output=$("$list_devices_script" 2>&1); then
        if echo "$output" | grep -q "Found Orbbec device"; then
            log "✅ 检测到奥比相机:"
            echo "$output" | grep "Found Orbbec device" | while read line; do
                log "  $line"
            done
            HEAD_ORBBEC_CAMERA_EXIST=1
            return 0
        else
            error "❌ 未检测到奥比相机"
            HEAD_CAMERA_TYPE=rs_camera
            HEAD_ORBBEC_CAMERA_EXIST=0
            return 0
        fi
    else
        error "❌ 奥比相机检测脚本执行失败: $output"
        return 1
    fi
}

# 检查RealSense相机环境变量
check_realsense_env() {
    log "检查RealSense相机环境变量..."
    
    # 不再检查腕部相机环境变量，只检查是否有RealSense相机
    log "✅ 跳过腕部相机环境变量检查，直接启动头部RealSense相机"
    return 0
}

# 检查RealSense相机是否存在
check_realsense_camera() {
    log "检查RealSense相机是否存在..."
    
    source ~/.bashrc
    # 检查pyrealsense2包是否安装
    if ! python3 -c "import pyrealsense2" 2>/dev/null; then
        log "❌ 未安装pyrealsense2包，开始安装"

        if ! python3 -m pip install pyrealsense2 -i https://pypi.tuna.tsinghua.edu.cn/simple 2>/dev/null; then
            error "❌ 安装pyrealsense2包失败"
            return 1
        fi

        if ! python3 -c "import pyrealsense2" 2>/dev/null; then
            error "❌ 安装pyrealsense2包失败"
            return 1
        fi
        return 0
    fi


    # 检查左腕相机序列号环境变量
    if [ -z "$LEFT_WRIST_CAMERA_SERIAL_NO" ]; then
        python3 $WORKSPACE_DIR/src/kuavo_camera/scripts/scan_realsence.py
        source ~/.bashrc
    fi

    # 检查左腕相机序列号环境变量是否存在
    if [ -n "$LEFT_WRIST_CAMERA_SERIAL_NO" ]; then
        log "✅ 检测到左腕相机序列号: $LEFT_WRIST_CAMERA_SERIAL_NO"
        HAS_LEFT_WRIST=1
        
    fi

    if [ -n "$RIGHT_WRIST_CAMERA_SERIAL_NO" ]; then
        log "✅ 检测到右腕相机序列号: $RIGHT_WRIST_CAMERA_SERIAL_NO"
        HAS_RIGHT_WRIST=1
    fi

    # 检查USB设备中的RealSense相机（只考虑头部相机）
    local realsense_devices=$(lsusb 2>/dev/null | grep -i "RealSense.*Depth Camera" || true)
    
    if [ -n "$realsense_devices" ]; then
        log "✅ 检测到RealSense头部相机:"
        echo "$realsense_devices" | while read line; do
            log "  $line"
        done
        HEAD_RS_CAMERA_EXIST=1
        return 0
    else
        log "未检测到 RealSense 头部相机"
        return 0
    fi
}


# 启动相机驱动
start_camera() {
    log "启动相机驱动..."

    source /opt/ros/noetic/setup.bash
    cd $WORKSPACE_DIR
    source ./devel/setup.bash
    
    # 停止可能冲突的进程
    log "停止可能冲突的RealSense进程..."
    pkill -f "realsense2_camera" 2>/dev/null || true
    pkill -f "realsense2_camera_manager" 2>/dev/null || true
    sleep 2
    
    # 再检查环境变量
    if ! check_realsense_env; then
        warn "⚠️  RealSense相机环境变量未配置，尝试启动但可能失败"
    fi
    
    # 启动相机节点（直接使用SDK包）
    log "启动相机节点..."
    roslaunch kuavo_camera cameras.launch head_camera_type:=$HEAD_CAMERA_TYPE has_left_wrist:=$HAS_LEFT_WRIST has_right_wrist:=$HAS_RIGHT_WRIST rviz:=0 >/dev/null 2>&1 &
    CAMERA_PID=$!
    
    
    # 等待相机启动
    sleep 3
    
    # 根据相机存在情况等待相应话题
    log "等待相机话题启动..."
    
    # 使用Python rospy等待话题
    python3 << EOF
import rospy
import sys
import time

# 初始化ROS节点
rospy.init_node('topic_wait_node', anonymous=True)

# 获取环境变量
head_orbbec_exist = int('$HEAD_ORBBEC_CAMERA_EXIST')
head_rs_exist = int('$HEAD_RS_CAMERA_EXIST')
has_left_wrist = int('$HAS_LEFT_WRIST')
has_right_wrist = int('$HAS_RIGHT_WRIST')

# 等待头部相机话题
if head_orbbec_exist == 1 or head_rs_exist == 1:
    print("等待头部相机话题...")
    try:
        rospy.wait_for_message('/head_camera/color/image_raw', rospy.AnyMsg, timeout=20.0)
        print("✅ 头部相机话题已启动")
    except rospy.ROSException as e:
        print(f"⚠️  头部相机话题启动超时: {e}")

# 等待左手腕相机话题
if has_left_wrist == 1:
    print("等待左手腕相机话题...")
    try:
        rospy.wait_for_message('/left_wrist_camera/color/image_raw', rospy.AnyMsg, timeout=20.0)
        print("✅ 左手腕相机话题已启动")
    except rospy.ROSException as e:
        print(f"⚠️  左手腕相机话题启动超时: {e}")

# 等待右手腕相机话题
if has_right_wrist == 1:
    print("等待右手腕相机话题...")
    try:
        rospy.wait_for_message('/right_wrist_camera/color/image_raw', rospy.AnyMsg, timeout=20.0)
        print("✅ 右手腕相机话题已启动")
    except rospy.ROSException as e:
        print(f"⚠️  右手腕相机话题启动超时: {e}")

print("话题等待完成")
EOF
    
    # 检查相机节点是否运行
    if rosnode list 2>/dev/null | grep -q "realsense2_camera"; then
        log "✅ RealSense相机驱动启动成功"
        return 0
    else
        warn "⚠️  RealSense相机驱动可能未启动"
        return 1
    fi
}

# 运行统一测试
run_unified_test() {
    log "运行统一相机测试..."
    cd "$SCRIPT_DIR"
    python3 camera_test_tool.py --project-dir=$WORKSPACE_DIR
    return $?
}

# 强制清理相机相关进程与节点
force_stop_all_cameras() {
    log "强制清理相机相关进程与节点..."
    
    # 先杀掉可能在后台运行的roslaunch，避免respawn（广泛匹配camera相关）
    pkill -f "roslaunch.*cameras.launch" 2>/dev/null || true
    pkill -f "roslaunch.*ob_camera.launch" 2>/dev/null || true
    pkill -f "roslaunch.*camera" 2>/dev/null || true
    pkill -f "roslaunch.*kuavo_camera" 2>/dev/null || true
    pkill -f "roslaunch.*realsense2_camera" 2>/dev/null || true
    
    # 杀掉典型进程名（双保险）
    pkill -f "realsense2_camera" 2>/dev/null || true
    pkill -f "realsense2_camera_manager" 2>/dev/null || true
    pkill -f "orbbec_camera" 2>/dev/null || true
    pkill -f "kuavo_camera" 2>/dev/null || true
    
    # 按命名空间杀ROS节点
    if command -v rosnode >/dev/null 2>&1; then
        # 先正常kill节点
        rosnode list 2>/dev/null | grep -E '^/(left_wrist_camera|right_wrist_camera|camera|head_camera)/' | xargs -r -n1 rosnode kill 2>/dev/null || true
        rosnode kill /camera_rviz 2>/dev/null || true
        
        # 直接kill相机节点的底层进程（避免respawn迅速复活）
        for n in $(rosnode list 2>/dev/null | grep -E '^/(left_wrist_camera|right_wrist_camera|camera|head_camera)/|^/camera_rviz$'); do
            pid=$(rosnode info "$n" 2>/dev/null | awk '/Pid:/ {print $2}')
            if [ -n "$pid" ]; then
                kill -9 "$pid" 2>/dev/null || true
            fi
        done
    fi
    
    sleep 2
}

# 主函数
main() {
    log "🚀 开始相机测试流程..."
    
    # 检查并安装依赖
    check_and_install_dependencies

    # 编译apriltag相关包
    compile_apriltag_packages || {
        error "apriltag相关包编译失败"
        exit 1
    }
    
    # 检查工作空间
    check_workspace || {
        error "工作空间检查失败"
        exit 1
    }
    
    # 启动前先清理可能残留的相机进程/节点
    # force_stop_all_cameras

    # 检查奥比相机
    check_orbbec_camera

    # 检查RealSense相机
    check_realsense_camera

    # 检查是否检测到头部相机
    if [ $HEAD_ORBBEC_CAMERA_EXIST -eq 0 ] && [ $HEAD_RS_CAMERA_EXIST -eq 0 ]; then
        error "❌ 未检测到任何头部相机设备"
        echo "请检查USB连接和相机电源"
        return 1
    fi

    # 启动RealSense相机驱动
    if ! start_camera; then
        error "相机启动失败"
        exit 1
    fi
    
    # 等待相机启动
    sleep 3
    
    # 运行统一测试
    run_unified_test
}

# 清理函数
cleanup() {
    log "清理资源..."
    # 使用更安全的方式停止相机驱动/roslaunch
    # force_stop_all_cameras
}

# 设置信号处理
# trap cleanup EXIT INT TERM

# 运行主函数
main 
