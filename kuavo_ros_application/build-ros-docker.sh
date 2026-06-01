#!/bin/bash

# Navigate to the script directory
cd "$(dirname "$0")"

# Build ROS Noetic Docker image with NVIDIA GPU support
echo "Building ROS Noetic Docker image..."
docker build -f Dockerfile.ros-noetic -t ros-noetic-nvidia:latest .

echo ""
echo "============================================"
echo "Build completed!"
echo "============================================"
echo ""
echo "To run the container with workspace mounted:"
echo "  ./run-ros-docker.sh"
echo ""
echo "Or manually run:"
echo "  docker run --gpus all -it --rm \\"
echo "    --network host \\"
echo "    -v /media/data/Documents/kuavo_ros_application:/catkin_ws/src \\"
echo "    -w /catkin_ws \\"
echo "    ros-noetic-nvidia:latest"
