#!/bin/bash

set -e

# 日志函数
log() {
    echo "[`date '+%F %T'`] $1"
}

# 获取当前目录路径
CURRENT_DIR=$(pwd)

# 规则文件路径
RULES_FILE="/etc/udev/rules.d/99-check_5G_up.rules"
# systemd 服务文件路径
SERVICE_FILE="/etc/systemd/system/check_5G_up.service"

# 检查规则文件是否存在
log "检查规则文件是否已存在：$RULES_FILE..."
if [ -f "$RULES_FILE" ]; then
    log "规则文件已存在，跳过创建步骤。"
else
    log "规则文件不存在，正在创建..."
    sudo tee "$RULES_FILE" > /dev/null <<EOF
ACTION=="add", SUBSYSTEM=="net", \
  ENV{ID_VENDOR_ID}=="2c7c", ENV{ID_MODEL_ID}=="0801", \
  ATTRS{manufacturer}=="Quectel", ATTRS{product}=="RM530N-GL", \
  NAME="wwan0", \
  ENV{SYSTEMD_WANTS}="check_5G_up.service"
EOF
    log "规则文件已创建：$RULES_FILE"
fi

# 重新加载 udev 服务以应用新的规则
log "重新加载 udev 服务..."
sudo udevadm control --reload

# 检查 systemd 服务文件是否存在
log "检查 systemd 服务文件是否已存在：$SERVICE_FILE..."
if [ -f "$SERVICE_FILE" ]; then
    log "systemd 服务文件已存在，跳过创建步骤。"
else
    log "systemd 服务文件不存在，正在创建..."
    sudo tee "$SERVICE_FILE" > /dev/null <<EOF
[Unit]
Description=Service triggered by udev
After=network.target

[Service]
Type=oneshot
ExecStart=/usr/bin/bash $CURRENT_DIR/check_5G_up.sh

[Install]
WantedBy=multi-user.target
EOF
    log "systemd 服务单元文件已创建：$SERVICE_FILE"
fi

# 重新加载 systemd 配置
log "重新加载 systemd 配置..."
sudo systemctl daemon-reload

# 检查 resolv.conf 中是否包含指定 nameserver 行
RESOLV_FILE="/etc/resolv.conf"
TARGET_DNS_LINE="nameserver 114.114.114.114 223.5.5.5"

log "检查 $RESOLV_FILE 中是否包含 DNS 配置行..."
if grep -Fxq "$TARGET_DNS_LINE" "$RESOLV_FILE"; then
    log "已包含指定 DNS 配置行，跳过追加。"
else
    log "未找到指定 DNS 配置行，正在追加..."
    echo "$TARGET_DNS_LINE" | sudo tee -a "$RESOLV_FILE" > /dev/null
    log "DNS 配置已追加到 $RESOLV_FILE"
fi


log "所有操作已完成！"
