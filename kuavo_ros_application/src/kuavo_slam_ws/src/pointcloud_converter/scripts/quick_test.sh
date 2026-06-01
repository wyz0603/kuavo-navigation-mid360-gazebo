#!/bin/bash

echo "🚀 快速测试点云转换器修复方案"
echo "=================================="

# # 检查ROS
# if ! pgrep -x "roscore" > /dev/null; then
#     echo "❌ 请先启动 roscore"
#     exit 1
# fi

echo "📋 当前点云相关话题:"
rostopic list | grep -E "(lidar|point|cloud)" | sort

echo ""
echo "🔍 检查关键话题状态:"

# 检查输入话题（Hesai驱动输出）
echo -n "  /lidar_points (Hesai输出): "
if rostopic info /lidar_points >/dev/null 2>&1; then
    echo "✅ 存在"
    echo "    发布频率: $(timeout 3 rostopic hz /lidar_points 2>/dev/null | grep "average rate" | awk '{print $3}' || echo "未知") Hz"
else
    echo "❌ 不存在 - 请检查Hesai驱动是否运行"
fi

# 检查输出话题（转换器输出）
echo -n "  /lidar_points_converted (转换器输出): "
if rostopic info /lidar_points_converted >/dev/null 2>&1; then
    echo "✅ 存在"
    echo "    发布频率: $(timeout 3 rostopic hz /lidar_points_converted 2>/dev/null | grep "average rate" | awk '{print $3}' || echo "未知") Hz"
else
    echo "❌ 不存在 - 转换器可能未运行"
fi

echo ""
echo "🔍 检查点云字段格式:"

# 检查输入点云字段
echo "  输入点云字段 (/lidar_points):"
if timeout 5 rostopic echo /lidar_points/fields -n 1 >/dev/null 2>&1; then
    timeout 5 rostopic echo /lidar_points/fields -n 1 | grep "name:" | awk '{print "    - " $2}'
else
    echo "    ❌ 无法获取字段信息"
fi

# 检查输出点云字段
echo "  输出点云字段 (/lidar_points_converted):"
if timeout 5 rostopic echo /lidar_points_converted/fields -n 1 >/dev/null 2>&1; then
    timeout 5 rostopic echo /lidar_points_converted/fields -n 1 | grep "name:" | awk '{print "    - " $2}'
else
    echo "    ❌ 无法获取字段信息"
fi

echo ""
echo "📊 转换器统计信息:"
timeout 3 rostopic echo /rosout | grep "Conversion Stats" | tail -1 || echo "  未找到统计信息"

echo ""
echo "🎯 建议的下一步操作:"
echo "  1. 如果 /lidar_points 不存在，请启动Hesai驱动:"
echo "     roslaunch hesai_ros_driver start.launch"
echo ""
echo "  2. 如果 /lidar_points_converted 不存在，请启动转换器:"
echo "     roslaunch pointcloud_converter simple_converter.launch"
echo ""
echo "  3. 启动完整的建图系统:"
echo "     roslaunch pointcloud_converter build_map_fixed.launch"
echo ""
echo "=================================="
