#!/bin/bash
set -e

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
SERVICE_DIR=$(dirname $SCRIPT_DIR)/humanoid_websocket_sdk_server/service
START_WS_SERVER_NODE_SERVICE=$SERVICE_DIR/start_websocket_server_node.service
START_WS_SERVER_NODE=$SCRIPT_DIR/start_websocket_server.sh
KUAVO_ROS_APPLICATION_WS_PATH=$(dirname $(dirname $(dirname $SCRIPT_DIR)))


echo "KUAVO_ROS_APPLICATION_WS_PATH: $KUAVO_ROS_APPLICATION_WS_PATH"
echo "SERVICE_DIR: $SERVICE_DIR"
echo "START_WS_SERVER_NODE: $START_WS_SERVER_NODE"


source /opt/ros/noetic/setup.bash
cd $KUAVO_ROS_APPLICATION_WS_PATH
catkin build kuavo_msgs ocs2_msgs


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

sed -i "s|^Environment=ROS_MASTER_URI=.*|Environment=ROS_MASTER_URI=$ROS_MASTER_URI|" $START_WS_SERVER_NODE_SERVICE
sed -i "s|^Environment=ROS_IP=.*|Environment=ROS_IP=$ROS_IP|" $START_WS_SERVER_NODE_SERVICE
sed -i "s|^Environment=KUAVO_ROS_APPLICATION_WS_PATH=.*|Environment=KUAVO_ROS_APPLICATION_WS_PATH=$KUAVO_ROS_APPLICATION_WS_PATH|" $START_WS_SERVER_NODE_SERVICE
sed -i "s|^ExecStart=.*|ExecStart=$START_WS_SERVER_NODE|" $START_WS_SERVER_NODE_SERVICE


sudo cp $START_WS_SERVER_NODE_SERVICE /etc/systemd/system/
sudo systemctl daemon-reload

read -p "Do you want to enable websocket server service to start on boot? (y/n): " enable_response
case $enable_response in
    [Yy]* )
        sudo systemctl enable start_websocket_server_node.service
        echo "Service enabled successfully"
        ;;
    * )
        echo "Skipping service enable"
        ;;
esac

read -p "Do you want to start websocket server service now? (y/n): " start_response
case $start_response in
    [Yy]* )
        sudo systemctl start start_websocket_server_node.service
        echo "Service started successfully"
        ;;
    * )
        echo "Skipping service start"
        ;;
esac

echo
echo "Note: Some changes (udev rules and service autostart) will take effect after system reboot."
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