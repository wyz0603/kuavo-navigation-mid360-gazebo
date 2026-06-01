#!/bin/bash

# Get the directory where this script is located
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

echo "Starting ROS Noetic container with workspace mounted..."
echo "Workspace: $SCRIPT_DIR"
echo ""

docker run --runtime=nvidia -it --rm \
  --name ros-noetic-dev \
  --network host \
  --privileged \
  -v "$SCRIPT_DIR":/catkin_ws \
  -w /catkin_ws \
  ros-noetic-nvidia:latest

# Note: If you need GUI support (RViz, rqt), use run-ros-docker-gui.sh instead
