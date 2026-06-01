#!/bin/bash

echo "开始监测5G模块状态"

echo "开始监测拨号结果状态..."

COUNT=0

while true; do
    status_output=$(/bin/systemctl status ModemManager.service)

    # 检查是否包含 QMI IPv4 Settings
    qmi_line=$(echo "$status_output" | grep "QMI IPv4 Settings:")
    if [[ -n "$qmi_line" ]]; then
        echo "✅ 检测到 QMI IPv4 Settings:"
        echo "$qmi_line"
        break
    fi

    # 检查是否包含 reject cause
    reject_line=$(echo "$status_output" | grep "reject cause: cs-domain-not-available")
    if [[ -n "$reject_line" ]]; then
        echo "⚠️ 检测到 reject cause: cs-domain-not-available，5G模块拨号失败，正在尝试重启 ModemManager..."
        sudo /bin/systemctl restart ModemManager.service
        COUNT=0
	sleep 3
        continue
    fi

    echo "未检测到拨号结果状态，5 秒后重试..."
    ((COUNT++))
    if ((COUNT > 20));then
	    echo "监测超时，退出监测！"
	    break
    fi
    /bin/sleep 5
done

