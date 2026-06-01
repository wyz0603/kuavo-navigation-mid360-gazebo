#!/bin/bash

# 获取传入的参数
IP=$1

# 检查是否有参数传入
if [ -z "$IP" ]; then
    # 如果没有传入参数，检查 /etc/hosts 中是否有 'kuavo_master'
    if grep -q "kuavo_master" /etc/hosts; then
        # 获取 'kuavo_master' 的当前 IP 地址
        current_ip=$(grep "kuavo_master" /etc/hosts | awk '{print $1}')
        
        # 如果当前 IP 不是 127.0.0.1，则更新为 127.0.0.1
        if [ "$current_ip" != "127.0.0.1" ]; then
            sudo sed -i "s/^$current_ip  kuavo_master/127.0.0.1  kuavo_master/" /etc/hosts
            echo "已将 kuavo_master 的 IP 地址更新为 127.0.0.1"
        else
            echo "kuavo_master 的 IP 地址已经是 127.0.0.1，跳过更新"
        fi
    else
        # 如果 /etc/hosts 中没有 'kuavo_master'，则添加一条新的记录
        echo "127.0.0.1  kuavo_master" | sudo tee -a /etc/hosts > /dev/null
        echo "已添加 127.0.0.1  kuavo_master 到 /etc/hosts"
    fi
else
    # 如果有传入参数，检查 /etc/hosts 中是否有 'kuavo_master'
    # 判断当前设备是否为DHCP主机
    if ip a | grep -q "192.168.26.1"; then
        # 当前设备是DHCP主机，目标IP应为192.168.26.12
        TARGET_IP="192.168.26.12"
    else
        # 当前设备不是DHCP主机，目标IP应为192.168.26.1
        TARGET_IP="192.168.26.1"
    fi
    
    if [[ "$IP" == "$TARGET_IP" ]]; then
        echo "IP 匹配成功: $IP"
    else
        echo "IP 匹配失败！期望的IP是 $TARGET_IP，但提供的是 $IP。请联系技术支持检查上下位机 DHCP 配置！"
        return 1
    fi
    if grep -q "kuavo_master" /etc/hosts; then
        # 如果有 'kuavo_master'，修改其对应的 IP 地址
        sudo sed -i "s/^[^#]*kuavo_master.*$/$IP  kuavo_master/" /etc/hosts
        echo "已更新 kuavo_master 的 IP 地址为 $IP"
    else
        # 如果没有 'kuavo_master'，添加一条新的记录
        echo "$IP  kuavo_master" | sudo tee -a /etc/hosts > /dev/null
        echo "已添加 $IP  kuavo_master 到 /etc/hosts"
    fi
fi
