#!/usr/bin/env sh
SCRIPT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)

# Default values
DEFAULT_REPOSITORY_NAME="kuavo_slam_opensource_img"
DEFAULT_TAG="0.1.0"

# Parse command line arguments
while [ $# -gt 0 ]; do
    case "$1" in
        --name)
            REPOSITORY_NAME="$2"
            shift 2
            ;;
        --tag)
            TAG="$2"
            shift 2
            ;;
        *)
            echo "Unknown option: $1"
            exit 1
            ;;
    esac
done

# Use default values if not provided
REPOSITORY_NAME=${REPOSITORY_NAME:-$DEFAULT_REPOSITORY_NAME}
TAG=${TAG:-$DEFAULT_TAG}
echo "REPOSITORY_NAME: $REPOSITORY_NAME"
echo "TAG: $TAG"
cd "$SCRIPT_DIR" || exit
sudo rm -rf .tls
cp -r ../src/ros-foxglove-bridge/tls .tls
docker build -t $REPOSITORY_NAME:$TAG .
if [ $? -ne 0 ]; then
    echo "Failed to build docker image $REPOSITORY_NAME:$TAG"
    exit 1
fi
echo "Successfully built docker image $REPOSITORY_NAME:$TAG"
cd -
exit 0