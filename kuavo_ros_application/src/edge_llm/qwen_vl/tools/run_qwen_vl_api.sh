#!/bin/bash

# Qwen2.5-VL-7B-Instruct API 服务启动脚本

set -e

CONTAINER_NAME="qwen_vl_api"
IMAGE="qwen-vl-api:20251010"
PORT=9000

echo "========================================"
echo "Qwen2.5-VL-7B-Instruct API 服务"
echo "========================================"
echo "容器: $CONTAINER_NAME"
echo "镜像: $IMAGE"
echo "端口: $PORT"
echo "========================================"

# 检查容器是否已在运行
if [ "$(sudo docker ps -q -f name=^$CONTAINER_NAME$)" ]; then
    echo "容器已在运行，正在停止..."
    sudo docker stop $CONTAINER_NAME
    sleep 2
fi

# 删除旧容器
if [ "$(sudo docker ps -aq -f name=^$CONTAINER_NAME$)" ]; then
    sudo docker rm $CONTAINER_NAME
fi

# 运行容器
echo "正在启动 API 服务器..."
sudo docker run -d \
  --name $CONTAINER_NAME \
  --runtime=nvidia \
  --ipc=host \
  --ulimit memlock=-1 \
  --ulimit stack=67108864 \
  -p $PORT:$PORT \
  -v /mnt/nvme/cache:/root/.cache \
  -v $(pwd)/qwen_vl_api.py:/workspace/qwen_vl_api.py \
  $IMAGE \
  bash -c "pip install -q flask transformers accelerate qwen-vl-utils && python3 /workspace/qwen_vl_api.py"

echo ""
echo "容器已启动！"
echo ""
echo "API 端点:"
echo "  - http://localhost:$PORT/health"
echo "  - http://localhost:$PORT/v1/chat/completions"
echo "  - http://localhost:$PORT/v1/models"
echo ""
echo "查看日志: sudo docker logs -f $CONTAINER_NAME"
echo "停止服务: sudo docker stop $CONTAINER_NAME"
echo "========================================"
