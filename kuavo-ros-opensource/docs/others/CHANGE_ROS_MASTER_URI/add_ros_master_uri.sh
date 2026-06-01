#!/bin/bash

# 接收一个参数，判断是否是 "body"
PARAM=$1

# 如果参数是 "body"，则判断是否已设置 ROS_HOSTNAME
if [ "$PARAM" == "body" ]; then
    # 检查 /root/.bashrc 是否包含 ROS_MASTER_URI

     if grep -q "^export ROS_MASTER_URI=" $HOME/.bashrc; then
        # 如果已经存在，则修改它的值
        sed -i 's|^export ROS_MASTER_URI=.*|export ROS_MASTER_URI=http://kuavo_master:11311|' $HOME/.bashrc
        echo "ROS_MASTER_URI 已更新为 http://kuavo_master:11311 在 $HOME/.bashrc 中"
    else
        # 如果不存在，则添加新的变量
        echo "export ROS_MASTER_URI=http://kuavo_master:11311" >> $HOME/.bashrc
        echo "ROS_MASTER_URI 已添加到 $HOME/.bashrc 中"
    fi

    # 检查 $HOME/.bashrc 是否包含 ROS_HOSTNAME
    if grep -q "^export ROS_HOSTNAME=" $HOME/.bashrc; then
        # 如果已经存在，则修改它的值
        sed -i 's|^export ROS_HOSTNAME=.*|export ROS_HOSTNAME=kuavo_master|' $HOME/.bashrc
        echo "ROS_HOSTNAME 已更新为 kuavo_master 在 $HOME/.bashrc 中"
    else
        # 如果不存在，则添加新的变量
        echo "export ROS_HOSTNAME=kuavo_master" >> $HOME/.bashrc
        echo "ROS_HOSTNAME 已添加到 $HOME/.bashrc 中"
    fi

    # 检查 $HOME/.bashrc 是否包含 ROS_IP
    if grep -q "^export ROS_IP=" $HOME/.bashrc; then
        # 如果已经存在，则修改它的值
        sed -i 's|^export ROS_IP=.*|export ROS_IP=kuavo_master|' $HOME/.bashrc
        echo "ROS_IP 已更新为 kuavo_master 在 $HOME/.bashrc 中"
    else
        # 如果不存在，则添加新的变量
        echo "export ROS_IP=kuavo_master" >> $HOME/.bashrc
        echo "ROS_IP 已添加到 $HOME/.bashrc 中"
    fi

    if sudo grep -q "^export ROS_MASTER_URI=" /root/.bashrc; then
        # 如果已经存在，则修改它的值
        sudo sed -i 's|^export ROS_MASTER_URI=.*|export ROS_MASTER_URI=http://kuavo_master:11311|' /root/.bashrc
        echo "ROS_MASTER_URI 已更新为 http://kuavo_master:11311 在 /root/.bashrc 中"
    else
        # 如果不存在，则添加新的变量
        sudo bash -c 'echo "export ROS_MASTER_URI=http://kuavo_master:11311" >> /root/.bashrc'
        echo "ROS_MASTER_URI 已添加到 /root/.bashrc 中"
    fi

    # 检查 /root/.bashrc 是否包含 ROS_HOSTNAME
    if sudo grep -q "^export ROS_HOSTNAME=" /root/.bashrc; then
        # 如果已经存在，则修改它的值
        sudo sed -i 's|^export ROS_HOSTNAME=.*|export ROS_HOSTNAME=kuavo_master|' /root/.bashrc
        echo "ROS_HOSTNAME 已更新为 kuavo_master 在 /root/.bashrc 中"
    else
        # 如果不存在，则添加新的变量
        sudo bash -c 'echo "export ROS_HOSTNAME=kuavo_master" >> /root/.bashrc'
        echo "ROS_HOSTNAME 已添加到 /root/.bashrc 中"
    fi

    # 检查 /root/.bashrc 是否包含 ROS_IP
    if sudo grep -q "^export ROS_IP=" /root/.bashrc; then
        # 如果已经存在，则修改它的值
        sudo sed -i 's|^export ROS_IP=.*|export ROS_IP=kuavo_master|' /root/.bashrc
        echo "ROS_IP 已更新为 kuavo_master 在 /root/.bashrc 中"
    else
        # 如果不存在，则添加新的变量
        sudo bash -c 'echo "export ROS_IP=kuavo_master" >> /root/.bashrc'
        echo "ROS_IP 已添加到 /root/.bashrc 中"
    fi
    export ROS_IP=kuavo_master
    export ROS_HOSTNAME=kuavo_master
    echo "已完成 $HOME/.bashrc 和 /root/.bashrc 的环境变量配置"
else
    # 检查 $HOME/.bashrc 是否包含 ROS_MASTER_URI
    if grep -q "^export ROS_MASTER_URI=" $HOME/.bashrc; then
        # 如果已经存在，则修改它的值
        sed -i 's|^export ROS_MASTER_URI=.*|export ROS_MASTER_URI=http://kuavo_master:11311|' $HOME/.bashrc
        echo "ROS_MASTER_URI 已更新为 http://kuavo_master:11311 在 $HOME/.bashrc 中"
    else
        # 如果不存在，则添加新的变量
        echo "export ROS_MASTER_URI=http://kuavo_master:11311" >> $HOME/.bashrc
        echo "ROS_MASTER_URI 已添加到 $HOME/.bashrc 中"
    fi

    # 检查 /root/.bashrc 是否包含 ROS_MASTER_URI
    if sudo grep -q "^export ROS_MASTER_URI=" /root/.bashrc; then
        # 如果已经存在，则修改它的值
        sudo sed -i 's|^export ROS_MASTER_URI=.*|export ROS_MASTER_URI=http://kuavo_master:11311|' /root/.bashrc
        echo "ROS_MASTER_URI 已更新为 http://kuavo_master:11311 在 /root/.bashrc 中"
    else
        # 如果不存在，则添加新的变量
        sudo bash -c 'echo "export ROS_MASTER_URI=http://kuavo_master:11311" >> /root/.bashrc'
        echo "ROS_MASTER_URI 已添加到 /root/.bashrc 中"
    fi
    echo "已完成 $HOME/.bashrc 和 /root/.bashrc 的环境变量配置"
    # 获取192.168.26.xx网段的IP地址
    LOCAL_IP=$(ip addr | grep -oP '(?<=inet\s)192\.168\.26\.[0-9]+(?=/)' | head -n 1)
    
    # 如果没有找到192.168.26.xx网段的IP，则使用HOSTNAME
    if [ -z "$LOCAL_IP" ]; then
        LOCAL_IP=192.168.26.1
        echo "未找到192.168.26.xx网段的IP地址，使用 192.168.26.1 作为备选"
    else
        echo "找到192.168.26.xx网段的IP地址: $LOCAL_IP"
    fi

    # 检查 $HOME/.bashrc 是否包含 ROS_IP
    if grep -q "^export ROS_IP=" $HOME/.bashrc; then
        # 如果已经存在，则修改它的值
        sed -i 's|^export ROS_IP=.*|export ROS_IP='"$LOCAL_IP"'|' $HOME/.bashrc
        echo "ROS_IP 已更新为 $LOCAL_IP 在 $HOME/.bashrc 中"
    else
        # 如果不存在，则添加新的变量
        echo "export ROS_IP=$LOCAL_IP" >> $HOME/.bashrc
        echo "ROS_IP 已添加到 $HOME/.bashrc 中"
    fi
    # 检查 /root/.bashrc 是否包含 ROS_IP
    if sudo grep -q "^export ROS_IP=" /root/.bashrc; then
        # 如果已经存在，则修改它的值
        sudo sed -i 's|^export ROS_IP=.*|export ROS_IP='"$LOCAL_IP"'|' /root/.bashrc
        echo "ROS_IP 已更新为 $LOCAL_IP 在 /root/.bashrc 中"
    else
        # 如果不存在，则添加新的变量
        sudo bash -c 'echo "export ROS_IP='"$LOCAL_IP"'" >> /root/.bashrc'
        echo "ROS_IP 已添加到 /root/.bashrc 中"
    fi
fi

export ROS_MASTER_URI=http://kuavo_master:11311
