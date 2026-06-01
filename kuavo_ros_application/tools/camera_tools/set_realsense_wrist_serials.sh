#!/bin/bash
# 用法: bash set_realsense_wrist_serials.sh [true|false]
# true  -> 跳过获取，直接对调 ~/.bashrc 中的 LEFT/RIGHT WRIST
# false -> 自动获取 RealSense 相机序列号并写入 (默认)

set -euo pipefail

is_swap=${1:-false}
BASHRC_FILE="${HOME}/.bashrc"

if [[ "$is_swap" == "true" ]]; then
    echo "🔁 参数为 true，跳过获取，直接对调已有变量..."

    left_serial=$(grep -m1 '^export LEFT_WRIST_CAMERA_SERIAL_NO=' "$BASHRC_FILE" | cut -d= -f2-)
    right_serial=$(grep -m1 '^export RIGHT_WRIST_CAMERA_SERIAL_NO=' "$BASHRC_FILE" | cut -d= -f2-)

    if [[ -z "${left_serial:-}" || -z "${right_serial:-}" ]]; then
        echo "❌ 无法从 ~/.bashrc 中找到左右手腕相机序列号，请先运行一次: bash tools/camera_tools/set_realsense_wrist_serials.sh"
        exit 1
    fi

    sed -i '/LEFT_WRIST_CAMERA_SERIAL_NO/d' "$BASHRC_FILE"
    sed -i '/RIGHT_WRIST_CAMERA_SERIAL_NO/d' "$BASHRC_FILE"

    {
      echo "export LEFT_WRIST_CAMERA_SERIAL_NO=${right_serial}"
      echo "export RIGHT_WRIST_CAMERA_SERIAL_NO=${left_serial}"
    } >> "$BASHRC_FILE"

    echo "✅ 已对调并写入 ~/.bashrc"
    echo "🔁 执行 'source ~/.bashrc' 使其生效"
    exit 0
fi

echo "🔎 正在扫描 RealSense 设备..."

# 复用 scan_realsence.py 所依赖的 pyrealsense2 获取序列号
mapfile -t rs_serials < <(python3 - <<'PY'
import pyrealsense2 as rs

ctx = rs.context()
devices = ctx.query_devices()
for dev in devices:
    try:
        print(dev.get_info(rs.camera_info.serial_number))
    except Exception:
        pass
PY
)

if [[ ${#rs_serials[@]} -lt 2 ]]; then
    echo "❌ 检测到的 RealSense 序列号不足 2 个，请确认两台手腕相机已正确连接。"
    exit 1
fi

serial_left="${rs_serials[0]}"
serial_right="${rs_serials[1]}"

echo "✅ 获取到序列号:"
echo "   LEFT_WRIST  -> ${serial_left}"
echo "   RIGHT_WRIST -> ${serial_right}"

sed -i '/LEFT_WRIST_CAMERA_SERIAL_NO/d' "$BASHRC_FILE"
sed -i '/RIGHT_WRIST_CAMERA_SERIAL_NO/d' "$BASHRC_FILE"

{
  echo "export LEFT_WRIST_CAMERA_SERIAL_NO=\"${serial_left}\""
  echo "export RIGHT_WRIST_CAMERA_SERIAL_NO=\"${serial_right}\""
} >> "$BASHRC_FILE"

echo "✅ 已写入 ~/.bashrc"
echo "🔁 执行 'source ~/.bashrc' 使其生效"
