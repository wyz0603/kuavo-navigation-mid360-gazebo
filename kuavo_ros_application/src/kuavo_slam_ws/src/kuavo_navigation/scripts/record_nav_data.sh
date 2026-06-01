#!/bin/bash

SAVE_DIR=~/maps
BAG_PREFIX=$(date +%Y%m%d_%H%M%S)
BAG_FILE="${SAVE_DIR}/${BAG_PREFIX}.bag"

# 创建目录（如果不存在）
mkdir -p "$SAVE_DIR"

# 保留最近 50 个 .bag 文件，删除更老的
find "$SAVE_DIR" -maxdepth 1 -type f -name "*.bag" | sort | head -n -50 | xargs -r rm -v

# 获取 topic 列表
TOPICS=$(rosparam get /record_nav_data/topic_whitelist | sed 's/- //g')

# 检查 topic 是否为空
if [ -z "$TOPICS" ]; then
    echo "[ERROR] /record_nav_data/topic_whitelist is empty or missing!"
    exit 1
fi

# 启动 rosbag record
echo "[INFO] Recording to: $BAG_FILE"
rosbag record -O "$BAG_FILE" $TOPICS