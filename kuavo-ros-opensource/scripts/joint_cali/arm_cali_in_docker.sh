#!/bin/bash
xhost +
REAL_FLAG=${1:-""}

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PARENT_DIR="$(dirname "$SCRIPT_DIR")"
WORK_DIR="$(dirname "$PARENT_DIR")"
echo "WORK_DIR: $WORK_DIR"

WORK_DIR_IN_DOCKER=$WORK_DIR

docker run -it --rm \
  -e DISPLAY=$DISPLAY \
  -e ROBOT_VERSION=$ROBOT_VERSION \
  --net host \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v $HOME/.Xauthority:/root/.Xauthority \
  -v $WORK_DIR:$WORK_DIR_IN_DOCKER \
  -v $HOME/.config:/root/.config \
  joint_cali:latest bash -c "source $WORK_DIR_IN_DOCKER/devel/setup.bash && $WORK_DIR_IN_DOCKER/scripts/joint_cali/arm_cali.py $REAL_FLAG"
