#!/bin/bash

source /opt/ros/noetic/setup.bash

catkin build kuavo_msgs ocs2_msgs kuavo_tf2_web_republisher
source $KUAVO_ROS_APPLICATION_WS_PATH/devel/setup.bash
roslaunch kuavo_tf2_web_republisher start_websocket_server.launch