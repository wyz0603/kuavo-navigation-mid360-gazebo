#!/bin/bash

# 用法说明
if [ $# -lt 1 ]; then
    echo "用法: $0 <输入bag文件>"
    echo "示例: $0 input.bag"
    exit 1
fi

INPUT_BAG=$1
BASE_NAME=$(basename "$INPUT_BAG" .bag)
DIR_NAME=$(dirname "$INPUT_BAG")

# 定义两组固定话题
TOPICS_GROUP1=("/kuavo_arm_traj" "/sensors_data_raw")
TOPICS_GROUP2=("/leju_quest_bone_poses" "/recording_status" "/recording_message" "/quest_joystick_data" "/quest_hand_finger_tf")

# 为第一组话题创建输出文件
OUTPUT_BAG1="${DIR_NAME}/${BASE_NAME}_group1.bag"
FILTER1="topic == '${TOPICS_GROUP1[0]}' or topic == '${TOPICS_GROUP1[1]}'"

echo "提取中..."
echo "从 $INPUT_BAG 中提取以下话题:"
for t in "${TOPICS_GROUP1[@]}"; do
    echo " - $t"
done
echo "输出文件: $OUTPUT_BAG1"

rosbag filter "$INPUT_BAG" "$OUTPUT_BAG1" "$FILTER1"

if [ $? -eq 0 ]; then
    echo "✅ 第一组话题提取成功！"
else
    echo "❌ 第一组话题提取失败，请检查参数或原始 bag 文件是否损坏。"
fi

# 为第二组话题创建输出文件
OUTPUT_BAG2="${DIR_NAME}/${BASE_NAME}_group2.bag"
FILTER2="topic == '${TOPICS_GROUP2[0]}' or topic == '${TOPICS_GROUP2[1]}' or topic == '${TOPICS_GROUP2[2]}' or topic == '${TOPICS_GROUP2[3]}' or topic == '${TOPICS_GROUP2[4]}'"

echo "提取中..."
echo "从 $INPUT_BAG 中提取以下话题:"
for t in "${TOPICS_GROUP2[@]}"; do
    echo " - $t"
done
echo "输出文件: $OUTPUT_BAG2"

rosbag filter "$INPUT_BAG" "$OUTPUT_BAG2" "$FILTER2"

if [ $? -eq 0 ]; then
    echo "✅ 第二组话题提取成功！"
else
    echo "❌ 第二组话题提取失败，请检查参数或原始 bag 文件是否损坏。"
fi
