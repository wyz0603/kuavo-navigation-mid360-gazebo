#!/bin/bash

# 获取当前时间戳
timestamp=$(date +"%Y-%m-%d_%H-%M-%S")

# 创建目录（如果不存在）
mkdir -p ~/maps
mkdir -p ~/pcd

# 保存地图
rosrun map_server map_saver -f ~/maps/map_"$timestamp"

# 修复 YAML 中的 nan 为 0.0
yaml_file=~/maps/map_"$timestamp".yaml
if grep -q "nan" "$yaml_file"; then
    sed -i 's/nan/0.0/g' "$yaml_file"
    echo "🔧 已修复 YAML 文件中的 'nan'：$yaml_file"
else
    echo "✅ YAML 文件中没有 'nan'：$yaml_file"
fi

echo "✅ 地图已保存到 ~/maps/map_$timestamp.pgm 和 .yaml"

# 检查并重命名临时数据库文件
temp_db_path=~/maps/temp_task_points.db
if [ -f "$temp_db_path" ]; then
    new_db_path=~/maps/map_"$timestamp"_task_points.db
    mv "$temp_db_path" "$new_db_path"
    echo "✅ 临时数据库已重命名为：$new_db_path"
    
    # 使用Python脚本更新数据库中的map_name字段
    map_name="map_$timestamp"
    script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
    python3 "$script_dir/update_task_points_db.py" "$new_db_path" "$map_name"
else
    echo "ℹ️ 没有找到临时数据库文件：$temp_db_path"
fi

# 获取 PCD 文件路径
pcd_filepath=$(rospack find fast_lio)/PCD/scans.pcd

# 等待 PCD 文件生成（最多等待30秒）
echo "⏳ 等待 PCD 文件生成..."
timeout=30
while [ ! -f "$pcd_filepath" ] && [ $timeout -gt 0 ]; do
    sleep 1
    timeout=$((timeout-1))
    echo -n "."
    
    # 检查并杀死 laserMapping 节点
    if pgrep -f "laserMapping" > /dev/null; then
        echo ""
        echo "🔍 检测到建图节点正在运行，正在发送 SIGINT 信号... 停止建图"
        pkill -SIGINT -f "laserMapping"
        sleep 2
        echo "✅ 建图节点已收到 SIGINT 信号"
    fi
done
echo ""

# 检查 PCD 文件是否存在并复制
if [ -f "$pcd_filepath" ]; then
    cp "$pcd_filepath" ~/maps/temp_map_"$timestamp".pcd
    sleep 1
    rosrun kuavo_mapping downsample_pcd.py --pcd ~/maps/temp_map_$timestamp.pcd --output_file ~/maps/map_$timestamp.pcd
    echo "✅ PCD 文件已保存到 ~/maps/map_$timestamp.pcd"
else
    echo "❌ PCD 文件不存在: $pcd_filepath"
    echo "⚠️ 请确保地图构建完成后再运行此脚本"
fi