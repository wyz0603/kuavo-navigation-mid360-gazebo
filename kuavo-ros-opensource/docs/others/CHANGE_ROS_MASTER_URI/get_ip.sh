#!/bin/bash

# 获取所有物理网卡的 IP 地址，并筛选符合 192.168.26.xx 范围的 IP
ip addr show | grep -oP '(?<=inet\s)192\.168\.26\.\d+' | while read -r ip; do
    iface=$(ip -o -4 addr show | grep "$ip" | awk '{print $2}')
    echo "$ip"
done
