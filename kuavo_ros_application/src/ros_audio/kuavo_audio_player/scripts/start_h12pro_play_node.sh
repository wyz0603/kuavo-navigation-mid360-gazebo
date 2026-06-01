#!/bin/bash

source /opt/ros/noetic/setup.bash
source $KUAVO_ROS_APPLICATION_WS_PATH/devel/setup.bash
KUAVO_AUDIO_PLAYER_DIR=$(rospack find kuavo_audio_player)

cd $KUAVO_AUDIO_PLAYER_DIR
echo "KUAVO_ROS_APPLICATION_WS_PATH: $KUAVO_ROS_APPLICATION_WS_PATH"
echo "KUAVO_AUDIO_PLAYER_DIR: $KUAVO_AUDIO_PLAYER_DIR"
echo "ROS_MASTER_URI: $ROS_MASTER_URI"
echo "ROS_IP: $ROS_IP"
echo "CAMERA_TYPE: $CAMERA_TYPE"

sudo alsa force-reload

handle_exit() {
    echo "Stopping all nodes..."
    pkill -P $$ 
    exit 0
}

check_node() {
    if rosnode list | grep -q "/humanoid_plan_arm_trajectory_node"; then
        echo "Node /humanoid_plan_arm_trajectory_node is running."
        return 0 
    else
        echo "Node /humanoid_plan_arm_trajectory_node is not running."
        return 1 
    fi
}

check_video_node(){
    if rosnode list | grep -q "/rosout"; then
        echo "Node /rosout is running,roscore has running."
        return 0 
    else
        echo "Node /rosout is not running,roscore has not running"
        return 1 
    fi
    echo 
}


get_joystick_type() {
    joystick_type=$(rosparam get /joystick_type)
    echo "Joystick type: $joystick_type"
}


trap "handle_exit" SIGTERM SIGINT SIGHUP SIGQUIT

nodes_started=false
camera_started=false

while true; do
    if check_node; then
        get_joystick_type
        if [ "$joystick_type" == "h12" ] && [ "$nodes_started" == false ]; then
            roslaunch kuavo_audio_player play_music.launch &
            nodes_started=true 
        elif [ "$joystick_type" != "h12" ] && [ "$nodes_started" == true ]; then
            echo "Joystick type is not h12. Stopping nodes..."
            pkill -f "play_music.launch"
            nodes_started=false 
        fi
    fi
    if check_video_node; then
        if [ "$camera_started" == false ] && [ "$CAMERA_TYPE" != "none" ]; then
            if [ "$CAMERA_TYPE" == "realsense" ]; then
                roslaunch realsense2_camera rs_camera.launch &
                camera_started=true
            elif [ "$CAMERA_TYPE" == "orbbec" ]; then
                roslaunch kuavo_camera cameras.launch head_camera_type:=orbbec_camera has_left_wrist:=false has_right_wrist:=false rviz:=false &
                camera_started=true
            else
                echo "Camera type is not realsense or orbbec. Skipping camera start."
                camera_started=false
            fi
        fi
    fi
    sleep 1 
done
