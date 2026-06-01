#!/bin/bash

# 获取当前脚本所在的绝对路径
SCRIPT_DIR=$(dirname "$(readlink -f "$0")")
WORKSPACE_ROOT=$(dirname "$(dirname "$(dirname "$SCRIPT_DIR")")")
PACKAGE_NAME="h12pro_controller_node"

# 检查是否已编译
DEVEL_LIB_PATH="$WORKSPACE_ROOT/devel/lib/python3/dist-packages/$PACKAGE_NAME"
if [ -d "$DEVEL_LIB_PATH" ]; then
    echo -e "\033[32m包 $PACKAGE_NAME 已编译，无需重复编译\033[0m"
    exit 0
fi

# 进入工作空间并编译（日志完全隐藏）
cd "$WORKSPACE_ROOT" || {
    echo -e "\033[31m错误：无法进入工作空间 $WORKSPACE_ROOT\033[0m"
    exit 1
}

echo -e "\033[33m开始编译包 $PACKAGE_NAME...\033[0m"
catkin build "$PACKAGE_NAME" -j$(nproc) >/dev/null 2>&1  # 关键：重定向所有输出

# 仅显示最终结果，无中间日志
if [ $? -eq 0 ]; then
    echo -e "\033[32m包 $PACKAGE_NAME 编译成功！\033[0m"
    exit 0
else
    echo -e "\033[31m包 $PACKAGE_NAME 编译失败！请检查日志文件\033[0m"
    exit 1
fi