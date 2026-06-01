#!/bin/bash

echo "🔍 开始调试点云话题..."
echo "请确保Hesai驱动正在运行"
echo ""

# 检查ROS是否运行
if ! pgrep -x "roscore" > /dev/null; then
    echo "❌ ROS Master未运行，请先启动roscore"
    exit 1
fi

# 列出所有话题
echo "📋 当前所有话题:"
rostopic list | grep -E "(lidar|velodyne|point|cloud)" || echo "未找到相关话题"
echo ""

# 检查特定话题
echo "🔍 检查关键话题:"
for topic in "/lidar_points" "/velodyne_points" "lidar_points" "velodyne_points"; do
    echo -n "  $topic: "
    if rostopic info "$topic" >/dev/null 2>&1; then
        echo "✅ 存在"
        echo "    类型: $(rostopic type "$topic")"
        echo "    发布者: $(rostopic info "$topic" | grep "Publishers:" -A 1 | tail -1 | xargs)"
        echo "    订阅者: $(rostopic info "$topic" | grep "Subscribers:" -A 1 | tail -1 | xargs)"
    else
        echo "❌ 不存在"
    fi
done

echo ""
echo "🔍 运行Python调试脚本..."
python3 src/kuavo_slam_ws/src/pointcloud_converter/scripts/debug_topics.py
