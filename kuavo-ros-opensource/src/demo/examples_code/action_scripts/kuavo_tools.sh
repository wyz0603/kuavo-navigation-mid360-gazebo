#!/bin/bash

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
BLUE='\033[0;34m'
NC='\033[0m' # 无颜色

# 获取脚本所在目录
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# 获取当前用户名并判断是上位机还是下位机
current_user=$(whoami)
if [ "$current_user" == "lab" ]; then
    machine_type="下位机"
elif [ "$current_user" == "kuavo" ]; then
    machine_type="上位机"
else
    machine_type="未知设备"
fi

echo -e "${BLUE}当前用户: ${GREEN}$current_user ${BLUE}(${GREEN}$machine_type${BLUE})${NC}"

# 检测Git分支和最新提交日期
check_git_repo() {
    local repo_path="$1"
    local repo_name="$2"
    
    if [ -d "$repo_path/.git" ]; then
        cd "$repo_path"
        local branch=$(git branch --show-current)
        local last_commit_date=$(git log -1 --format=%cd --date=format:"%Y-%m-%d %H:%M:%S")
        echo -e "${BLUE}仓库 ${GREEN}$repo_name${BLUE} 分支: ${GREEN}$branch${BLUE}, 最新提交: ${GREEN}$last_commit_date${NC}"
        return 0
    else
        return 1
    fi
}

# 检测Git仓库
echo -e "\n${YELLOW}Git仓库信息:${NC}"
if [ "$machine_type" == "下位机" ]; then
    # 检测下位机的仓库
    if ! check_git_repo "$HOME/kuavo-ros-control" "kuavo-ros-control"; then
        # 如果检测不到kuavo-ros-control，不输出错误信息
        :
    fi
    
    if ! check_git_repo "$HOME/kuavo-ros-opensource" "kuavo-ros-opensource"; then
        echo -e "${RED}错误: 未检测到 kuavo-ros-opensource 仓库，此仓库为客户使用必需${NC}"
    fi
elif [ "$machine_type" == "上位机" ]; then
    # 检测上位机的仓库
    if ! check_git_repo "$HOME/kuavo_ros_application" "kuavo_ros_application"; then
        echo -e "${RED}错误: 未检测到 kuavo_ros_application 仓库${NC}"
    fi
fi

# 检测环境变量ROBOT_VERSION（仅下位机）
if [ "$machine_type" == "下位机" ]; then
    echo -e "\n${YELLOW}机器人版本信息:${NC}"
    if [ -n "$ROBOT_VERSION" ]; then
        echo -e "${BLUE}ROBOT_VERSION: ${GREEN}$ROBOT_VERSION${NC}"
    else
        echo -e "${RED}错误: 未设置 ROBOT_VERSION 环境变量${NC}"
    fi
fi

# 功能选择菜单
show_menu() {
echo -e "${BLUE}\n重要提示: 确保遥控器服务已经启动在执行本脚本，遥控器服务参考:${NC}https://kuavo.lejurobot.com/manual/basic_usage/kuavo-ros-control/docs/5%E5%8A%9F%E8%83%BD%E6%A1%88%E4%BE%8B/%E9%80%9A%E7%94%A8%E6%A1%88%E4%BE%8B/H12%E9%81%A5%E6%8E%A7%E5%99%A8%E4%BD%BF%E7%94%A8%E5%BC%80%E5%8F%91%E6%A1%88%E4%BE%8B/${NC}"
    echo -e "\n${YELLOW}功能选择:${NC}"
    echo -e "${BLUE}1. 动作语音出厂化设置${NC}"
    echo -e "${BLUE}2. 自定义动作文件设置${NC}"
    echo -e "${BLUE}0. 退出${NC}"
    echo -n -e "${YELLOW}请选择功能 [0-2]: ${NC}"
    read choice
    
    case $choice in
        1)
            echo -e "${GREEN}执行动作语音出厂化设置...${NC}"
            python3 "${SCRIPT_DIR}/factory_reset_action.py"
            ;;
        2)
            echo -e "${GREEN}执行自定义动作文件设置...${NC}"
            python3 "${SCRIPT_DIR}/custom_action.py"
            ;;
        0)
            echo -e "${GREEN}退出程序${NC}"
            exit 0
            ;;
        *)
            echo -e "${RED}无效选择，请重新输入${NC}"
            show_menu
            ;;
    esac
}

# 显示功能菜单
show_menu