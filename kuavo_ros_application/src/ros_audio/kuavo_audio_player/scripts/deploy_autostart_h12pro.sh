#!/bin/bash
set -e

# 解析命令行参数
CI_MODE=false
for arg in "$@"; do
  if [ "$arg" == "-ci" ]; then
    CI_MODE=true
    echo "CI 模式已启用，将跳过所有交互式命令"
  fi
done

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
SERVICE_DIR=$(dirname $SCRIPT_DIR)/service
START_H12PRO_PLAY_NODE_SERVICE=$SERVICE_DIR/start_h12pro_play_node.service
START_H12PRO_PLAY_NODE=$SCRIPT_DIR/start_h12pro_play_node.sh
KUAVO_ROS_APPLICATION_WS_PATH=$(dirname $(dirname $(dirname $(dirname $SCRIPT_DIR))))


echo "KUAVO_ROS_APPLICATION_WS_PATH: $KUAVO_ROS_APPLICATION_WS_PATH"
echo "SERVICE_DIR: $SERVICE_DIR"
echo "START_H12PRO_PLAY_NODE: $START_H12PRO_PLAY_NODE"

CURRENT_USER="${SUDO_USER:-$(id -un)}"
if [ -z "$CURRENT_USER" ]; then
  echo "警告：无法检测到当前用户，将回退为 kuavo"
  CURRENT_USER="kuavo"
fi
echo "将把 service 的 User 设置为: $CURRENT_USER"

# 询问相机型号
    echo
    echo "请选择相机型号："
    echo "1. realsense"
    echo "2. orbbec"
    echo "3. 不启用相机"
    read -p "请输入数字 (1 或 2 或 3): " camera_choice

    case $camera_choice in
        1)
            CAMERA_MODEL="realsense"
            ;;
        2)
            CAMERA_MODEL="orbbec"
            ;;
        3)
            CAMERA_MODEL="none"
            ;;
        *)
            echo "无效的输入，不启用相机"
            CAMERA_MODEL="none"
            ;;
    esac

echo "已选择相机型号: $CAMERA_MODEL"


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
cd $KUAVO_ROS_APPLICATION_WS_PATH
catkin build apriltag_ros
catkin build 


if [ -z "$ROS_MASTER_URI" ]; then
    ROS_MASTER_URI="http://localhost:11311"
    echo "ROS_MASTER_URI is empty, using default: $ROS_MASTER_URI"
fi

if [ -z "$ROS_IP" ]; then
    ROS_IP="127.0.0.1"
    echo "ROS_IP is empty, using default: $ROS_IP"
fi

echo "Current ROS_MASTER_URI: $ROS_MASTER_URI"
echo "Current ROS_IP: $ROS_IP"

sed -i "s|^Environment=ROS_MASTER_URI=.*|Environment=ROS_MASTER_URI=$ROS_MASTER_URI|" $START_H12PRO_PLAY_NODE_SERVICE
sed -i "s|^Environment=ROS_IP=.*|Environment=ROS_IP=$ROS_IP|" $START_H12PRO_PLAY_NODE_SERVICE
sed -i "s|^Environment=CAMERA_TYPE=.*|Environment=CAMERA_TYPE=$CAMERA_MODEL|" $START_H12PRO_PLAY_NODE_SERVICE
sed -i "s|^Environment=KUAVO_ROS_APPLICATION_WS_PATH=.*|Environment=KUAVO_ROS_APPLICATION_WS_PATH=$KUAVO_ROS_APPLICATION_WS_PATH|" $START_H12PRO_PLAY_NODE_SERVICE
sed -i "s|^ExecStart=.*|ExecStart=$START_H12PRO_PLAY_NODE|" $START_H12PRO_PLAY_NODE_SERVICE
sed -i "s|^User=.*|User=$CURRENT_USER|" $START_H12PRO_PLAY_NODE_SERVICE


sudo cp $START_H12PRO_PLAY_NODE_SERVICE /etc/systemd/system/
sudo systemctl daemon-reload

if [ "$CI_MODE" == "false" ]; then
    read -p "Do you want to enable h12pro play service to start on boot? (y/n): " enable_response
    case $enable_response in
        [Yy]* )
            sudo systemctl enable start_h12pro_play_node.service
            echo "Service enabled successfully"
            ;;
        * )
            echo "Skipping service enable"
            ;;
    esac

    read -p "Do you want to start h12pro play service now? (y/n): " start_response
    case $start_response in
        [Yy]* )
            sudo systemctl start start_h12pro_play_node.service
            echo "Service started successfully"
            ;;
        * )
            echo "Skipping service start"
            ;;
    esac
fi

echo
echo "Note: Some changes (udev rules and service autostart) will take effect after system reboot."
if [ "$CI_MODE" == "false" ]; then
    read -p "Do you want to reboot the system now? (y/n): " reboot_response
    case $reboot_response in
        [Yy]* )
            echo "System will reboot in 5 seconds..."
            sleep 5
            sudo reboot
            ;;
        * )
            echo "Skipping reboot. Please remember to reboot later for the changes to take full effect."
            ;;
    esac
fi