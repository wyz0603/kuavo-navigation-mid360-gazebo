#!/bin/bash

set -e

# 日志函数
log() {
    echo "[`date '+%F %T'`] $1"
}

log "🔍 检查网卡信息..."

# 自动查找网段为 192.168.26.X 的网卡
INTERFACE=$(ifconfig | grep -B1 "192.168.26" | head -n1 | awk -F: '{print $1}' | xargs)

if [ -z "$INTERFACE" ]; then
    log "❌ 没有找到匹配 192.168.26.X 网段的网卡，退出。"
    exit 1
fi

log "检测到目标网卡：$INTERFACE"

# 固定配置
SUBNET="192.168.26.0"
NETMASK="255.255.255.0"
ROUTER="192.168.26.1"
POOL_START="192.168.26.12"
POOL_END="192.168.26.12"
LEASE_TIME="600"

log "安装 isc-dhcp-server..."
sudo apt update
sudo apt install -y isc-dhcp-server

log "配置绑定网卡..."
sudo sed -i "s/^INTERFACESv4=\".*\"/INTERFACESv4=\"$INTERFACE\"/" /etc/default/isc-dhcp-server

# 检查 dhcpd.conf 文件是否存在
if [ -f "/etc/dhcp/dhcpd.conf" ]; then
    log "配置文件已存在，正在检查是否需要添加或修改配置..."

    # 检查是否已经包含相同的配置（通过匹配某些关键字）
    if grep -q "subnet $SUBNET netmask $NETMASK" /etc/dhcp/dhcpd.conf; then
        log "配置已存在，检查 range 配置..."

        # 检查是否包含正确的 range 配置
        if ! grep -q "range $POOL_START $POOL_END" /etc/dhcp/dhcpd.conf; then
            log "❌ 找到不匹配的 range 配置，正在修改为正确的 range：$POOL_START $POOL_END"
            # 使用 sed 修改 range 配置
            sudo sed -i "s/range .*/range $POOL_START $POOL_END;/g" /etc/dhcp/dhcpd.conf
            log "range 配置已更新为：$POOL_START $POOL_END"
        else
            log "range 配置正确，无需修改"
        fi
    else
        log "配置文件没有找到目标网段配置，正在追加新的配置..."
        # 如果没有配置，追加新配置
        sudo tee -a /etc/dhcp/dhcpd.conf > /dev/null <<EOF

subnet $SUBNET netmask $NETMASK {
  range $POOL_START $POOL_END;
  option routers $ROUTER;
  option subnet-mask $NETMASK;
}
EOF
        log "新的 DHCP 配置已添加到 /etc/dhcp/dhcpd.conf"
    fi
else
    log "配置文件不存在，正在创建新的 /etc/dhcp/dhcpd.conf..."

    # 创建并写入新的配置
    sudo tee /etc/dhcp/dhcpd.conf > /dev/null <<EOF
default-lease-time $LEASE_TIME;
max-lease-time $((LEASE_TIME * 2));
authoritative;

subnet $SUBNET netmask $NETMASK {
  range $POOL_START $POOL_END;
  option routers $ROUTER;
  option subnet-mask $NETMASK;
}
EOF
    log "新的 DHCP 配置已生成并写入 /etc/dhcp/dhcpd.conf"
fi

log "设置网卡 $INTERFACE 静态 IP 为 $ROUTER 并使其永久生效..."
sudo ip addr flush dev $INTERFACE
sudo ip addr add $ROUTER/24 dev $INTERFACE
sudo ip link set $INTERFACE up

log "启动 DHCP 服务..."
sudo systemctl restart isc-dhcp-server
sudo systemctl enable isc-dhcp-server

log "DHCP 服务部署完成，监听网卡：$INTERFACE，分配地址：$POOL_START"


# 创建或修改网络接口配置文件以实现永久生效
NETPLAN_FILE="/etc/netplan/99-$INTERFACE-static.yaml"
log "创建网络接口永久配置文件: $NETPLAN_FILE"
sudo tee $NETPLAN_FILE > /dev/null <<EOF
network:
  version: 2
  ethernets:
    $INTERFACE:
      addresses:
        - $ROUTER/24
      dhcp4: no
EOF
# 应用网络配置
log "应用网络配置..."
sudo netplan apply

# 确保所需的软件包已安装
log "确保所需的软件包已安装..."
sudo apt update
sudo apt install -y iptables iptables-persistent netfilter-persistent

log "配置 NAT 和 IP 转发，出口网口为 wwan0..."

# 启用 IP 转发
log "启用 IP 转发..."
sudo sysctl -w net.ipv4.ip_forward=1
# 确保 IP 转发在重启后仍然生效
if ! grep -q "net.ipv4.ip_forward=1" /etc/sysctl.conf; then
    echo "net.ipv4.ip_forward=1" | sudo tee -a /etc/sysctl.conf > /dev/null
    log "已将 IP 转发设置添加到 /etc/sysctl.conf"
fi

# 设置默认策略
log "设置默认策略..."
sudo iptables -P FORWARD DROP

# 配置 iptables NAT 规则
log "配置 iptables NAT 规则..."
# 检查并添加 NAT 规则，将来自 $INTERFACE 的流量通过 wwan0 转发出去
if ! sudo iptables -t nat -C POSTROUTING -o wwan0 -j MASQUERADE 2>/dev/null; then
    log "添加 MASQUERADE 规则..."
    sudo iptables -t nat -A POSTROUTING -o wwan0 -j MASQUERADE
else
    log "MASQUERADE 规则已存在，确保规则正确..."
    sudo iptables -t nat -D POSTROUTING -o wwan0 -j MASQUERADE
    sudo iptables -t nat -A POSTROUTING -o wwan0 -j MASQUERADE
fi

# 添加从DHCP服务器到wwan0的转发规则
if ! sudo iptables -t nat -C POSTROUTING -o wwan0 -j MASQUERADE 2>/dev/null; then
    log "添加DHCP服务器到wwan0的MASQUERADE规则..."
    sudo iptables -t nat -A POSTROUTING -o wwan0 -j MASQUERADE
fi

if ! sudo iptables -C FORWARD -i $INTERFACE -o wwan0 -j ACCEPT 2>/dev/null; then
    log "添加从 $INTERFACE 到 wwan0 的转发规则..."
    sudo iptables -A FORWARD -i $INTERFACE -o wwan0 -j ACCEPT
else
    log "从 $INTERFACE 到 wwan0 的转发规则已存在，确保规则正确..."
    sudo iptables -D FORWARD -i $INTERFACE -o wwan0 -j ACCEPT
    sudo iptables -A FORWARD -i $INTERFACE -o wwan0 -j ACCEPT
fi

if ! sudo iptables -C FORWARD -i wwan0 -o $INTERFACE -m state --state RELATED,ESTABLISHED -j ACCEPT 2>/dev/null; then
    log "添加从 wwan0 到 $INTERFACE 的回程规则..."
    sudo iptables -A FORWARD -i wwan0 -o $INTERFACE -m state --state RELATED,ESTABLISHED -j ACCEPT
else
    log "从 wwan0 到 $INTERFACE 的回程规则已存在，确保规则正确..."
    sudo iptables -D FORWARD -i wwan0 -o $INTERFACE -m state --state RELATED,ESTABLISHED -j ACCEPT
    sudo iptables -A FORWARD -i wwan0 -o $INTERFACE -m state --state RELATED,ESTABLISHED -j ACCEPT
fi

# 保存 iptables 规则以便重启后仍然生效
log "保存 iptables 规则..."
if command -v netfilter-persistent > /dev/null; then
    log "使用 netfilter-persistent 保存规则..."
    sudo netfilter-persistent save
    sudo netfilter-persistent reload
elif command -v iptables-save > /dev/null && command -v iptables-restore > /dev/null; then
    sudo iptables-save | sudo tee /etc/iptables.rules > /dev/null
    
    # 创建网络接口启动时加载规则的脚本
    IPTABLES_LOAD_FILE="/etc/network/if-pre-up.d/iptables"
    sudo tee $IPTABLES_LOAD_FILE > /dev/null <<EOF
#!/bin/sh
/sbin/iptables-restore < /etc/iptables.rules
exit 0
EOF
    sudo chmod +x $IPTABLES_LOAD_FILE
    log "已创建网络接口启动脚本以加载 iptables 规则"
else
    log "警告：未找到 iptables-save 或 iptables-restore 命令，无法持久化保存 iptables 规则"
fi

log "NAT 和 IP 转发配置完成，流量将通过 wwan0 转发"

# 检查系统中的其他网络接口
log "检查系统中的其他可用网络接口..."
# 获取所有网络接口，排除lo、$INTERFACE和wwan0
ALL_INTERFACES=$(ip -o link show | awk -F': ' '{print $2}')
OTHER_INTERFACES=""
for iface in $ALL_INTERFACES; do
    if [ "$iface" != "lo" ] && [ "$iface" != "$INTERFACE" ] && [ "$iface" != "wwan0" ]; then
        OTHER_INTERFACES="$OTHER_INTERFACES $iface"
    fi
done
OTHER_INTERFACES=$(echo $OTHER_INTERFACES | xargs)  # 去除多余空格

log "当前所有网络接口: $ALL_INTERFACES"
log "排除后的网络接口: $OTHER_INTERFACES"

if [ -n "$OTHER_INTERFACES" ]; then
    log "发现其他网络接口: $OTHER_INTERFACES"
    
    # 为每个其他网络接口配置NAT规则
    for iface in $OTHER_INTERFACES; do
        # 检查接口是否处于活动状态
        log "检查接口 $iface 的状态..."
        log "接口 $iface 的详细信息: $(ip link show $iface)"
        
        # 对于WiFi接口特殊处理
        if [[ "$iface" == wl* ]] || [[ "$iface" == wlp* ]]; then
            # 检查接口是否启用（UP）
            if ip link show $iface | grep -q "UP"; then
                log "WiFi接口 $iface 已启用，配置NAT规则..."
                log "为网络接口 $iface 配置NAT规则..."
            else
                log "WiFi接口 $iface 未启用，跳过配置"
                continue
            fi
        elif ip link show $iface | grep -q "state UP"; then
            log "接口 $iface 处于活动状态，开始配置NAT规则..."
            log "为网络接口 $iface 配置NAT规则..."
        else
            log "网络接口 $iface 当前未激活，跳过配置"
            continue
        fi
        
        # 添加MASQUERADE规则
        if ! sudo iptables -t nat -C POSTROUTING -o $iface -j MASQUERADE 2>/dev/null; then
            log "添加 $iface 的MASQUERADE规则..."
            sudo iptables -t nat -A POSTROUTING -o $iface -j MASQUERADE
        else
            log "$iface 的MASQUERADE规则已存在，确保规则正确..."
            sudo iptables -t nat -D POSTROUTING -o $iface -j MASQUERADE
            sudo iptables -t nat -A POSTROUTING -o $iface -j MASQUERADE
        fi
        
        # 添加从内部网络到外部网络的转发规则
        if ! sudo iptables -C FORWARD -i $INTERFACE -o $iface -j ACCEPT 2>/dev/null; then
            log "添加从 $INTERFACE 到 $iface 的转发规则..."
            sudo iptables -A FORWARD -i $INTERFACE -o $iface -j ACCEPT
        else
            log "从 $INTERFACE 到 $iface 的转发规则已存在，确保规则正确..."
            sudo iptables -D FORWARD -i $INTERFACE -o $iface -j ACCEPT
            sudo iptables -A FORWARD -i $INTERFACE -o $iface -j ACCEPT
        fi
        
        # 添加回程规则
        if ! sudo iptables -C FORWARD -i $iface -o $INTERFACE -m state --state RELATED,ESTABLISHED -j ACCEPT 2>/dev/null; then
            log "添加从 $iface 到 $INTERFACE 的回程规则..."
            sudo iptables -A FORWARD -i $iface -o $INTERFACE -m state --state RELATED,ESTABLISHED -j ACCEPT
        else
            log "从 $iface 到 $INTERFACE 的回程规则已存在，确保规则正确..."
            sudo iptables -D FORWARD -i $iface -o $INTERFACE -m state --state RELATED,ESTABLISHED -j ACCEPT
            sudo iptables -A FORWARD -i $iface -o $INTERFACE -m state --state RELATED,ESTABLISHED -j ACCEPT
        fi
        
        log "网络接口 $iface 的NAT规则配置完成"
    done
    
    log "所有可用网络接口的NAT规则配置完成"
else
    log "未发现其他可用的网络接口"
fi
