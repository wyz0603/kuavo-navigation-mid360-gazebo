#!/bin/bash

# DHCP 配置文件路径
DHCPD_CONF="/etc/dhcp/dhcpd.conf"

# 确保配置文件存在
if [ ! -f "$DHCPD_CONF" ]; then
    echo "错误: DHCP 配置文件 $DHCPD_CONF 不存在！请联系技术支持操作！"
    exit 1
fi

# 读取 DHCP 配置文件内容，并筛选出属于 192.168.26.xx 网段且在 subnet 内的 range
inside_subnet=false
ranges=()

while IFS= read -r line; do
    # 检测 subnet 192.168.26.0 相关配置块
    if [[ "$line" =~ ^[[:space:]]*subnet[[:space:]]+192\.168\.26\.0[[:space:]]+netmask ]]; then
        inside_subnet=true
    elif [[ "$line" =~ ^[[:space:]]*subnet ]]; then
        inside_subnet=false
    fi

    # 解析 range 192.168.26.xx xx.xx
    if $inside_subnet && [[ "$line" =~ ^[[:space:]]*range[[:space:]]+192\.168\.26\.[0-9]+[[:space:]]+192\.168\.26\.[0-9]+ ]]; then
        # 去掉末尾的分号
        range=$(echo "$line" | sed -E 's/^[[:space:]]*range[[:space:]]+|;//g')
        ranges+=("$range")
    fi
done < "$DHCPD_CONF"

# 输出结果
if [ ${#ranges[@]} -gt 0 ]; then
    for range in "${ranges[@]}"; do
        echo "$range"
    done
else
    echo "未找到 DHCP 在 192.168.26.xx 网段的动态分配范围！"
fi
