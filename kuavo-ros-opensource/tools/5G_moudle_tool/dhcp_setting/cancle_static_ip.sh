#!/bin/bash

set -e

# 日志函数
log() {
    echo "[`date '+%F %T'`] $1"
}

# 检查并停止 DHCP 服务
log "检查 DHCP 服务状态..."
if systemctl is-active --quiet isc-dhcp-server; then
    log "正在停止 DHCP 服务..."
    sudo systemctl stop isc-dhcp-server
    sudo systemctl disable isc-dhcp-server
    log "✅ DHCP 服务已停止并禁用"
else
    log "DHCP 服务未运行，无需停止"
fi

# 检查是否存在 DHCP 配置文件
if [ -f "/etc/dhcp/dhcpd.conf" ]; then
    log "备份 DHCP 配置文件..."
    sudo cp /etc/dhcp/dhcpd.conf /etc/dhcp/dhcpd.conf.bak
    log "✅ DHCP 配置文件已备份"
fi

# 清除 DHCP 相关的 iptables 规则
log "清除 DHCP 相关的 iptables 规则..."
sudo iptables -t nat -F POSTROUTING 2>/dev/null || log "清除 POSTROUTING 链失败，可能不存在"
sudo iptables -F FORWARD 2>/dev/null || log "清除 FORWARD 链失败，可能不存在"
log "✅ iptables 规则已清除"

# 保存 iptables 规则
log "保存 iptables 规则更改..."
if command -v netfilter-persistent > /dev/null; then
    sudo netfilter-persistent save
    sudo netfilter-persistent reload
    log "✅ 使用 netfilter-persistent 保存了 iptables 规则"
elif command -v iptables-save > /dev/null; then
    sudo iptables-save | sudo tee /etc/iptables.rules > /dev/null
    log "✅ 使用 iptables-save 保存了 iptables 规则"
else
    log "⚠️ 未找到保存 iptables 规则的工具，规则可能在重启后丢失"
fi


log "🔍 检查网卡信息..."

# 查找IP为192.168.26.1的网卡
INTERFACE=$(ip addr show | grep -B2 "inet 192.168.26.1" | head -n1 | awk '{print $2}' | sed 's/://')

if [ -z "$INTERFACE" ]; then
    log "❌ 没有找到IP为192.168.26.1的网卡，退出。"
    exit 1
fi

log "✅ 检测到目标网卡：$INTERFACE，当前IP为192.168.26.1"

# 检查网络配置方式（netplan或interfaces）
if [ -d "/etc/netplan" ]; then
    log "系统使用netplan管理网络配置"
    
    # 查找包含该网卡配置的netplan文件
    NETPLAN_FILE=$(grep -l "$INTERFACE" /etc/netplan/*.yaml 2>/dev/null || echo "")
    
    if [ -n "$NETPLAN_FILE" ]; then
        log "找到网卡配置文件: $NETPLAN_FILE，正在修改..."
        
        # 备份原配置文件
        sudo cp "$NETPLAN_FILE" "${NETPLAN_FILE}.bak"
        
        # 修改配置为DHCP
        sudo sed -i "/$INTERFACE:/,/dhcp4:/c\\    $INTERFACE:\\n      dhcp4: true" "$NETPLAN_FILE"
        
        log "应用新的网络配置..."
        sudo netplan apply
    else
        log "未找到网卡配置文件，创建新的配置..."
        
        # 创建新的netplan配置文件
        NEW_NETPLAN_FILE="/etc/netplan/99-$INTERFACE-dhcp.yaml"
        sudo tee "$NEW_NETPLAN_FILE" > /dev/null <<EOF
network:
  version: 2
  ethernets:
    $INTERFACE:
      dhcp4: true
EOF
        log "应用新的网络配置..."
        sudo netplan apply
    fi
    
elif [ -f "/etc/network/interfaces" ]; then
    log "系统使用interfaces管理网络配置"
    
    # 备份原配置文件
    sudo cp "/etc/network/interfaces" "/etc/network/interfaces.bak"
    
    # 检查是否已有该网卡的配置
    if grep -q "iface $INTERFACE" /etc/network/interfaces; then
        log "找到网卡配置，正在修改为DHCP..."
        
        # 修改为DHCP配置，只保留auto配置
        sudo sed -i "/iface $INTERFACE/,/^$/c\\auto $INTERFACE\\n" /etc/network/interfaces
    else
        log "未找到网卡配置，添加DHCP配置..."
        
        # 添加DHCP配置，只保留auto配置
        echo -e "\nauto $INTERFACE" | sudo tee -a /etc/network/interfaces > /dev/null
    fi
    
    log "重启网络服务..."
    sudo systemctl restart networking
else
    log "⚠️ 未找到支持的网络配置系统，尝试直接配置..."
fi

# 无论使用哪种配置方式，都直接使用ip命令临时配置
log "临时将网卡 $INTERFACE 设置为DHCP模式..."
sudo ip addr flush dev $INTERFACE

log "✅ 网卡 $INTERFACE 已成功配置为DHCP模式"
log "当前网卡IP信息:"
ip addr show $INTERFACE | grep "inet " | awk '{print $2}'
# 检查 /etc/hosts 文件中 kuavo_master 的配置
log "检查 /etc/hosts 文件中 kuavo_master 的配置..."
HOSTS_FILE="/etc/hosts"
KUAVO_MASTER_IP="192.168.26.1"
KUAVO_MASTER_HOSTNAME="kuavo_master"

if grep -q "$KUAVO_MASTER_HOSTNAME" "$HOSTS_FILE"; then
    # 找到 kuavo_master 配置，检查 IP 是否为 192.168.26.1
    CURRENT_IP=$(grep "$KUAVO_MASTER_HOSTNAME" "$HOSTS_FILE" | awk '{print $1}')
    
    if [ "$CURRENT_IP" != "$KUAVO_MASTER_IP" ]; then
        log "发现 kuavo_master 配置的 IP 为 $CURRENT_IP，正在更新为 $KUAVO_MASTER_IP..."
        sudo sed -i "s/^.*$KUAVO_MASTER_HOSTNAME.*$/$KUAVO_MASTER_IP $KUAVO_MASTER_HOSTNAME/" "$HOSTS_FILE"
        log "已将 kuavo_master 的 IP 更新为 $KUAVO_MASTER_IP"
    else
        log "kuavo_master 配置正确，IP 为 $KUAVO_MASTER_IP"
    fi
else
    # 未找到 kuavo_master 配置，添加新配置
    log "未找到 kuavo_master 配置，正在添加..."
    echo "$KUAVO_MASTER_IP $KUAVO_MASTER_HOSTNAME" | sudo tee -a "$HOSTS_FILE" > /dev/null
    log "已添加 kuavo_master 配置：$KUAVO_MASTER_IP $KUAVO_MASTER_HOSTNAME"
fi