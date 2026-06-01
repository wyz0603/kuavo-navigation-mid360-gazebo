#!/usr/bin/env bash
# 一键测试：启动录音节点 → 调用服务 → 检查结果

WORKSPACE_ROOT=~/kuavo_ros_application
PKG_NAME=kuavo_audio_recorder
NODE_SCRIPT=ros_aidio_record.py
SCRIPT_DIR="${WORKSPACE_ROOT}/src/ros_audio/kuavo_audio_recorder/scripts"

TEST_MUSIC_NUM="test001"
TEST_DURATION=5

# 1. Source ROS env
source /opt/ros/noetic/setup.bash
source "${WORKSPACE_ROOT}/devel/setup.bash"

# 2. Launch recording node
echo "🔊 启动录音节点 ${NODE_SCRIPT}"
rosrun ${PKG_NAME} ${NODE_SCRIPT} &
NODE_PID=$!
sleep 2

# 3. Call service with correct YAML syntax
echo "📡 调用 /record_music 服务 (music_number=${TEST_MUSIC_NUM}, time_out=${TEST_DURATION})"
rosservice call /record_music "{music_number: '${TEST_MUSIC_NUM}', time_out: ${TEST_DURATION}}"
echo "⌛ 等待录音结束…"
sleep $((TEST_DURATION + 1))

# 4. Check result file
LATEST=$(ls -t ${SCRIPT_DIR}/${TEST_MUSIC_NUM}_*.wav 2>/dev/null | head -1)
if [ -z "$LATEST" ]; then
  echo "❌ 未检测到录音文件!"
else
  echo "✅ 找到录音文件：$LATEST"
  stat --printf="文件: %n\n大小: %s 字节\n" "$LATEST"
fi

# 5. Cleanup
echo "🛑 停止录音节点 (PID=$NODE_PID)"
kill ${NODE_PID}

echo "🎯 测试完成！"
