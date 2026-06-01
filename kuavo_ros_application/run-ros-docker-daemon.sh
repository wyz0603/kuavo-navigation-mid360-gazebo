#!/bin/bash

# Get the directory where this script is located
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

echo "Starting ROS Noetic container in detached mode..."
echo "Workspace: $SCRIPT_DIR"
echo ""

# Stop and remove existing container if running
docker stop ros-noetic-dev 2>/dev/null || true
docker rm ros-noetic-dev 2>/dev/null || true

# Run container in detached mode
docker run --runtime=nvidia -d   --name ros-noetic-dev   --network host   --privileged   -v "$SCRIPT_DIR":/catkin_ws   -w /catkin_ws   ros-noetic-nvidia:latest   tail -f /dev/null

echo "Container started successfully!"
echo "To enter the container, use: docker exec -it ros-noetic-dev bash"
