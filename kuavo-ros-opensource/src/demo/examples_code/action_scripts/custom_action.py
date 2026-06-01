#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import json
import shutil
import subprocess
import logging
from pathlib import Path
import time

# 配置日志
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
    handlers=[
        logging.StreamHandler(),
        logging.FileHandler('/tmp/custom_action.log')
    ]
)
logger = logging.getLogger(__name__)

# 文件路径配置
LOWER_ACTION_PATH = "/home/lab/.config/lejuconfig/action_files"
UPPER_VOICE_PATH = "/home/kuavo/.config/lejuconfig/music"
CONFIG_JSON_PATH = "/home/lab/kuavo-ros-opensource/src/humanoid-control/h12pro_controller_node/config/customize_config.json"

def check_user():
    """检查当前用户，确定是上位机还是下位机"""
    user = os.environ.get('USER')
    if user == 'lab':
        return "下位机"
    elif user == 'kuavo':
        return "上位机"
    else:
        return "未知"

def create_directory(path):
    """创建目录（如果不存在）"""
    try:
        os.makedirs(path, exist_ok=True)
        logger.info(f"确保目录存在: {path}")
        return True
    except Exception as e:
        logger.error(f"创建目录失败 {path}: {e}")
        return False

def open_file_manager(path):
    """打开文件管理器到指定路径"""
    try:
        if os.path.exists(path):
            # 尝试使用不同的文件管理器
            file_managers = ["nautilus", "thunar", "pcmanfm", "dolphin", "xdg-open"]
            
            for manager in file_managers:
                try:
                    subprocess.Popen([manager, path])
                    logger.info(f"使用 {manager} 打开文件夹: {path}")
                    return True
                except FileNotFoundError:
                    continue
            
            logger.error("未找到可用的文件管理器")
            return False
        else:
            logger.error(f"路径不存在: {path}")
            return False
    except Exception as e:
        logger.error(f"打开文件管理器失败: {e}")
        return False

def wait_for_confirmation(message):
    """等待用户确认"""
    while True:
        response = input(f"{message} (y/n): ").strip().lower()
        if response == 'y':
            return True
        elif response == 'n':
            return False
        else:
            print("请输入 y 或 n")

def edit_config_file():
    """使用gedit编辑配置文件"""
    try:
        # 确保配置文件存在
        config_dir = os.path.dirname(CONFIG_JSON_PATH)
        if not os.path.exists(config_dir):
            logger.error(f"配置文件目录不存在: {config_dir}")
            return False
        
        # 如果配置文件不存在，创建一个基本的配置文件
        if not os.path.exists(CONFIG_JSON_PATH):
            default_config = {
                "action_file_path": LOWER_ACTION_PATH,
                "voice_file_path": UPPER_VOICE_PATH,
                "actions": []
            }
            
            with open(CONFIG_JSON_PATH, 'w', encoding='utf-8') as f:
                json.dump(default_config, f, ensure_ascii=False, indent=2)
            
            logger.info(f"创建了默认配置文件: {CONFIG_JSON_PATH}")
        
        # 使用gedit打开配置文件
        subprocess.run(["gedit", CONFIG_JSON_PATH])
        logger.info(f"已使用gedit打开配置文件: {CONFIG_JSON_PATH}")
        return True
    except Exception as e:
        logger.error(f"编辑配置文件失败: {e}")
        return False

def check_tact_robot_type(file_path):
    """检查tact文件中的robotType"""
    try:
        with open(file_path, 'r') as f:
            data = json.load(f)
            robot_type = data.get('robotType')
            return robot_type
    except Exception as e:
        logger.error(f"读取tact文件失败: {e}")
        return None

def process_action_files(action_path):
    """处理动作文件，检查robotType与环境变量是否匹配"""
    try:
        # 获取环境变量中的机器人版本
        env_robot_version = os.environ.get('ROBOT_VERSION')
        if not env_robot_version:
            logger.error("未设置环境变量 ROBOT_VERSION")
            raise SystemExit("未设置环境变量 ROBOT_VERSION，请先设置正确的机器人版本")

        tact_files = [f for f in os.listdir(action_path) if f.endswith('.tact')]
        version_mismatch = False

        for tact_file in tact_files:
            file_path = os.path.join(action_path, tact_file)
            robot_type = check_tact_robot_type(file_path)
            
            # 检查版本匹配情况
            if robot_type == '45' and env_robot_version != '45':
                logger.error(f"文件 {tact_file} 为45版本，但环境变量 ROBOT_VERSION={env_robot_version}")
                version_mismatch = True
            elif robot_type in ['41', '42'] and env_robot_version == '45':
                logger.error(f"文件 {tact_file} 为41/42版本，但环境变量 ROBOT_VERSION={env_robot_version}")
                version_mismatch = True
            elif not robot_type and env_robot_version == '45':
                logger.error(f"文件 {tact_file} 未指定版本，当前环境为45版本，可能存在兼容性问题")
            
            # 记录版本信息
            if robot_type is None or robot_type in ['41', '42']:
                logger.info(f"文件 {tact_file} 使用41/42版本或未指定版本")
            elif robot_type == '45':
                logger.info(f"文件 {tact_file} 使用45版本")
            else:
                logger.warning(f"文件 {tact_file} 使用未知版本: {robot_type}")

        if version_mismatch:
            raise SystemExit("存在版本不匹配的动作文件，请检查文件版本或环境变量设置")

    except Exception as e:
        logger.error(f"处理动作文件失败: {e}")
        raise SystemExit(f"处理动作文件失败: {e}")

def main():
    """主函数"""
    try:
        logger.info("启动自定义动作文件设置程序")
        
        # 检查用户类型
        user_type = check_user()
        logger.info(f"当前用户类型: {user_type}")
        print(f"当前用户类型: {user_type}")
        
        # 第一阶段：打开文件夹让用户放置文件
        if user_type == "下位机":
            # 创建动作文件目录
            if not create_directory(LOWER_ACTION_PATH):
                logger.error("创建动作文件目录失败")
                print("错误: 创建动作文件目录失败")
                return 1
            
            print(f"\n第一阶段: 请将您的自定义动作文件放入以下目录:")
            print(f"  {LOWER_ACTION_PATH}")
            
            # 打开文件管理器
            if not open_file_manager(LOWER_ACTION_PATH):
                print(f"无法自动打开文件管理器，请手动打开目录: {LOWER_ACTION_PATH}")
            
            # 等待用户确认文件放置完成
            if not wait_for_confirmation("\n您已经放置好动作文件了吗?"):
                print("操作已取消")
                return 0
            
            # 检查动作文件的robotType
            print("\n正在检查动作文件版本...")
            process_action_files(LOWER_ACTION_PATH)
            
            # 第二阶段：编辑配置文件
            print("\n第二阶段: 现在将打开配置文件进行编辑")
            print("请在配置文件中设置动作与语音的对应关系")
            print("配置文件格式示例:")
            print("""
{
  "action_file_path": "/home/lab/.config/lejuconfig/action_files",
  "voice_file_path": "/home/kuavo/.config/lejuconfig/music",
  "actions": [
    {
      "name": "挥手",
      "action_file": "1_挥手.tact",
      "voice_file": "1_hello.mp3"
    },
    {
      "name": "点赞",
      "action_file": "2_点赞.tact",
      "voice_file": "2_good.mp3"
    }
  ]
}
            """)
            
            time.sleep(2)  # 给用户时间阅读说明
            
            # 打开gedit编辑配置文件
            if not edit_config_file():
                print("错误: 无法打开配置文件进行编辑")
                return 1
            
            # 等待用户确认
            if not wait_for_confirmation("\n您已经完成配置文件的编辑了吗?"):
                print("操作已取消")
                return 0
            
            print("\n自定义动作设置已完成!")
            print("\n请注意: 您需要在上位机上放置对应的语音文件")
            print(f"上位机语音文件目录: {UPPER_VOICE_PATH}")
            
        elif user_type == "上位机":
            # 创建语音文件目录
            if not create_directory(UPPER_VOICE_PATH):
                logger.error("创建语音文件目录失败")
                print("错误: 创建语音文件目录失败")
                return 1
            
            print(f"\n请将您的自定义语音文件放入以下目录:")
            print(f"  {UPPER_VOICE_PATH}")
            
            # 打开文件管理器
            if not open_file_manager(UPPER_VOICE_PATH):
                print(f"无法自动打开文件管理器，请手动打开目录: {UPPER_VOICE_PATH}")
            
            # 等待用户确认
            if not wait_for_confirmation("\n您已经放置好语音文件了吗?"):
                print("操作已取消")
                return 0
            
            print("\n自定义语音设置已完成!")
            print("\n请注意: 您需要在下位机上设置动作与语音的对应关系")
            print(f"下位机动作文件目录: {LOWER_ACTION_PATH}")
            print(f"下位机配置文件路径: {CONFIG_JSON_PATH}")
            
        else:
            logger.error("未知的用户类型")
            print("错误: 未知的用户类型")
            return 1
        
        logger.info("自定义语音动作文件设置程序已完成")
        return 0

    except SystemExit as e:
        logger.error(f"程序终止: {e}")
        print(f"\n错误: {str(e)}")
        sys.exit(1)
    except Exception as e:
        logger.error(f"程序执行出错: {e}")
        print(f"\n错误: {str(e)}")
        sys.exit(1)

if __name__ == "__main__":
    sys.exit(main())