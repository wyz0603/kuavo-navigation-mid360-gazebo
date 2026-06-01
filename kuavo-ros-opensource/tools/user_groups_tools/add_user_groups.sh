#!/bin/bash

# 将 lab 用户和 root 用户加入 audio 用户组

# 检查 lab 用户是否已在 audio 用户组
if id -nG lab | grep -qw audio; then
    echo "lab 用户已经在 audio 用户组中，无需重复添加。"
else
    echo "正在将 lab 用户加入 audio 用户组..."
    if sudo usermod -aG audio lab; then
        echo "lab 用户已成功加入 audio 用户组。"
    else
        echo "lab 用户加入 audio 用户组失败！"
        exit 1
    fi
fi

# 检查 root 用户是否已在 audio 用户组
if id -nG root | grep -qw audio; then
    echo "root 用户已经在 audio 用户组中，无需重复添加。"
else
    echo "正在将 root 用户加入 audio 用户组..."
    if sudo usermod -aG audio root; then
        echo "root 用户已成功加入 audio 用户组。"
    else
        echo "root 用户加入 audio 用户组失败！"
        exit 1
    fi
fi

echo "操作完成。"
