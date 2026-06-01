#!/bin/bash

# DHCP 配置文件路径
DHCPD_CONF="/etc/dhcp/dhcpd.conf"

# 确保配置文件存在
if [ ! -f "$DHCPD_CONF" ]; then
    echo "错误: DHCP 配置文件 $DHCPD_CONF 不存在！"
    exit 1
fi

# 读取配置文件并修改范围
sudo sed -i 's/^\([ \t]*range[ \t]*\).*/\1 192.168.26.12 192.168.26.12;/g' /etc/dhcp/dhcpd.conf


echo "已将 DHCP 配置中的 range 更新为 192.168.26.12 192.168.26.12"
