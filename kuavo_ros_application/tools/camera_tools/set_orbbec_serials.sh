#!/bin/bash
# 用法: bash set_orbbec_serials.sh [true|false]
# true  -> 跳过获取，直接对调 ~/.bashrc 中的 HEAD/WAIST
# false -> 自动获取 Orbbec 相机序列号并写入 (默认)

is_swap=${1:-false}

if [[ "$is_swap" == "true" ]]; then
    echo "🔁 参数为 true，跳过获取，直接对调已有变量..."
    # 从 ~/.bashrc 中读取当前值
    head_serial=$(grep -m1 '^export HEAD_CAMERA_SERIAL_NO=' ~/.bashrc | cut -d= -f2)
    waist_serial=$(grep -m1 '^export WAIST_CAMERA_SERIAL_NO=' ~/.bashrc | cut -d= -f2)

    if [[ -z "$head_serial" || -z "$waist_serial" ]]; then
        echo "❌ 无法从 ~/.bashrc 中找到相机序列号，请先运行一次: bash set_orbbec_serials.sh false"
        exit 1
    fi

    # 删除旧定义
    sed -i '/HEAD_CAMERA_SERIAL_NO/d' ~/.bashrc
    sed -i '/WAIST_CAMERA_SERIAL_NO/d' ~/.bashrc

    # 写入对调后的定义
    {
      echo "export HEAD_CAMERA_SERIAL_NO=$waist_serial"
      echo "export WAIST_CAMERA_SERIAL_NO=$head_serial"
    } >> ~/.bashrc

    echo "✅ 已对调并写入 ~/.bashrc"
    echo "🔁 执行 'source ~/.bashrc' 使其生效"
    exit 0
fi

# =============== 正常模式：重新获取序列号 ===============

VID="2bc5"
declare -a ob_serials

echo "🔎 正在扫描 Orbbec 设备..."
for dev in /sys/bus/usb/devices/*; do
  if [[ -e "$dev/idVendor" ]]; then
    vid=$(cat "$dev/idVendor" 2>/dev/null)
    if [[ "$vid" == "$VID" ]]; then
      port=$(basename "$dev")
      product=$(cat "$dev/product" 2>/dev/null)
      serial=$(cat "$dev/serial" 2>/dev/null)
      echo "Found Orbbec device $product, usb port $port, serial number $serial"

      if [[ -n "$serial" ]]; then
        ob_serials+=("$serial")
      fi
    fi
  fi
done

if [[ ${#ob_serials[@]} -lt 2 ]]; then
    echo "❌ 检测到的 Orbbec 序列号不足 2 个，请确认两台相机已正确连接。"
    exit 1
fi

serial_head="${ob_serials[0]}"
serial_waist="${ob_serials[1]}"

echo "✅ 获取到序列号:"
echo "   HEAD -> $serial_head"
echo "   WAIST -> $serial_waist"

# 删除旧定义
sed -i '/HEAD_CAMERA_SERIAL_NO/d' ~/.bashrc
sed -i '/WAIST_CAMERA_SERIAL_NO/d' ~/.bashrc

# 默认第一个检测到的是头部相机
{
  echo "export HEAD_CAMERA_SERIAL_NO=$serial_head"
  echo "export WAIST_CAMERA_SERIAL_NO=$serial_waist"
} >> ~/.bashrc

echo "✅ 已写入 ~/.bashrc"
echo "🔁 执行 'source ~/.bashrc' 使其生效"
