#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import json
import shutil
import requests
import logging
import urllib.parse
import time

# 配置日志
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
    handlers=[
        logging.StreamHandler(),
        logging.FileHandler('/tmp/factory_reset_action.log')
    ]
)
logger = logging.getLogger(__name__)

# 基础URL
BASE_URL = "http://rustdesk.lejurobot.cn:8080/示例手臂动作音频文件/动作语音文件整合"
ENCODED_BASE_URL = urllib.parse.quote("http://rustdesk.lejurobot.cn:8080/示例手臂动作音频文件/动作语音文件整合", safe="/:").replace("%3A", ":")

# 文件路径配置
LOWER_ACTION_PATH = "/home/lab/.config/lejuconfig/action_files"
UPPER_VOICE_PATH = "/home/kuavo/.config/lejuconfig/music"
CONFIG_JSON_PATH = "/home/lab/kuavo-ros-opensource/src/humanoid-control/h12pro_controller_node/config/customize_config.json"

# 机器人版本映射
ROBOT_VERSION_MAP = {
    "41": "4代标准",
    "42": "4pro短手",
    "45": "4pro长手"
}

# 已知的文件列表（基于提供的目录结构）
KNOWN_FILES = {
    "4代标准": {
        "动作文件": ["1_挥手.tact", "2_抱拳.tact", "3_点赞.tact"],
        "customize_config.json": True
    },
    "4pro短手": {
        "动作文件": ["1_挥手.tact", "2_抱拳.tact", "3_点赞.tact"],
        "customize_config.json": True
    },
    "4pro长手": {
        "动作文件": ["1_挥手.tact", "2_抱拳.tact", "3_点赞.tact"],
        "customize_config.json": True
    },
    "语音文件": ["1_welcome.mp3", "2_hello.mp3", "3_good.mp3"]
}

def check_user():
    """检查当前用户类型"""
    username = os.getenv("USER")
    if username == "lab":
        return "下位机"
    elif username == "kuavo":
        return "上位机"
    else:
        return "未知"

def get_robot_version():
    """获取机器人版本"""
    version = os.getenv("ROBOT_VERSION")
    if version in ROBOT_VERSION_MAP:
        return version
    return None

def create_directory(directory):
    """创建目录（如果不存在）"""
    try:
        os.makedirs(directory, exist_ok=True)
        logger.info(f"创建目录: {directory}")
        return True
    except Exception as e:
        logger.error(f"创建目录失败: {directory}, 错误: {e}")
        return False

def download_file(url, target_path):
    """下载文件到指定路径"""
    try:
        logger.info(f"下载文件: {url} 到 {target_path}")
        response = requests.get(url, stream=True, timeout=30)
        
        if response.status_code == 404:
            logger.warning(f"文件不存在: {url}")
            return False
        
        response.raise_for_status()
        
        with open(target_path, 'wb') as f:
            for chunk in response.iter_content(chunk_size=8192):
                if chunk:
                    f.write(chunk)
        
        logger.info(f"文件下载成功: {target_path}")
        return True
    except Exception as e:
        logger.error(f"下载文件失败: {url}, 错误: {e}")
        return False

def download_config_and_actions(version_dir):
    """下载配置文件和动作文件"""
    success = True
    
    # 下载配置文件
    config_url = f"{BASE_URL}/{urllib.parse.quote(version_dir)}/customize_config.json"
    config_target = os.path.join(LOWER_ACTION_PATH, "customize_config.json")
    
    if not download_file(config_url, config_target):
        logger.error(f"下载配置文件失败: {config_url}")
        success = False
    
    # 下载动作文件 - 直接放在主目录下
    for action_file in KNOWN_FILES[version_dir]["动作文件"]:
        action_url = f"{BASE_URL}/{urllib.parse.quote(version_dir)}/{urllib.parse.quote('动作文件')}/{urllib.parse.quote(action_file)}"
        action_target = os.path.join(LOWER_ACTION_PATH, action_file)
        
        if not download_file(action_url, action_target):
            logger.warning(f"下载动作文件失败: {action_file}")
            success = False
        
        # 添加短暂延迟，避免请求过快
        time.sleep(0.5)
    
    return success

def download_music():
    """下载语音文件"""
    success = True
    
    # 下载语音文件 - 直接放在主目录下
    for voice_file in KNOWN_FILES["语音文件"]:
        voice_url = f"{BASE_URL}/{urllib.parse.quote('语音文件')}/{urllib.parse.quote(voice_file)}"
        voice_target = os.path.join(UPPER_VOICE_PATH, voice_file)
        
        if not download_file(voice_url, voice_target):
            logger.warning(f"下载语音文件失败: {voice_file}")
            success = False
        
        # 添加短暂延迟，避免请求过快
        time.sleep(0.5)
    
    return success

def update_config_json(version_dir):
    """更新配置文件"""
    try:
        # 源配置文件路径
        source_config = os.path.join(LOWER_ACTION_PATH, "customize_config.json")
        
        # 检查源配置文件是否存在
        if not os.path.exists(source_config):
            logger.error(f"源配置文件不存在: {source_config}")
            return False
        
        # 读取配置文件
        with open(source_config, 'r', encoding='utf-8') as f:
            config_data = json.load(f)
        
        # 更新动作文件路径，移除"动作文件/"前缀
        if "actions" in config_data:
            for action in config_data["actions"]:
                if "action_file" in action and action["action_file"].startswith("动作文件/"):
                    action["action_file"] = action["action_file"].replace("动作文件/", "")
                if "voice_file" in action and action["voice_file"].startswith("语音文件/"):
                    action["voice_file"] = action["voice_file"].replace("语音文件/", "")
        
        # 写入配置文件
        with open(CONFIG_JSON_PATH, 'w', encoding='utf-8') as f:
            json.dump(config_data, f, ensure_ascii=False, indent=2)
        
        logger.info(f"配置文件已更新: {CONFIG_JSON_PATH}")
        return True
    except Exception as e:
        logger.error(f"更新配置文件失败: {e}")
        return False

def main():
    """主函数"""
    logger.info("开始执行动作语音文件拉取程序")
    
    # 检查用户
    user_type = check_user()
    logger.info(f"当前用户类型: {user_type}")
    
    # 根据用户类型执行不同的操作
    if user_type == "下位机":
        # 获取机器人版本
        robot_version = get_robot_version()
        if not robot_version:
            logger.error("无法获取有效的机器人版本，程序退出")
            print("错误: 无法获取有效的机器人版本，请确保设置了ROBOT_VERSION环境变量")
            return 1
        
        version_dir = ROBOT_VERSION_MAP[robot_version]
        logger.info(f"机器人版本: {robot_version} ({version_dir})")
        print(f"机器人版本: {robot_version} ({version_dir})")
        
        # 创建下位机动作文件目录
        if not create_directory(LOWER_ACTION_PATH):
            logger.error("创建下位机动作文件目录失败，程序退出")
            print("错误: 创建下位机动作文件目录失败")
            return 1
        
        # 下载配置文件和动作文件
        print("开始下载配置文件和动作文件...")
        if not download_config_and_actions(version_dir):
            logger.warning("部分文件下载失败，但将继续执行")
            print("警告: 部分文件下载失败，但将继续执行")
        
        # 更新配置文件
        logger.info("开始更新配置文件")
        print("开始更新配置文件...")
        if not update_config_json(version_dir):
            logger.error("更新配置文件失败")
            print("错误: 更新配置文件失败")
            return 1
        
        print("\n下位机动作文件设置已完成!")
        print("\n请注意: 您需要在上位机上运行此程序，以下载对应的语音文件")
        print(f"上位机语音文件目录: {UPPER_VOICE_PATH}")
    
    elif user_type == "上位机":
        # 创建上位机语音文件目录
        if not create_directory(UPPER_VOICE_PATH):
            logger.error("创建上位机语音文件目录失败，程序退出")
            print("错误: 创建上位机语音文件目录失败")
            return 1
        
        # 下载语音文件
        print("开始下载语音文件...")
        if not download_music():
            logger.warning("部分语音文件下载失败，但将继续执行")
            print("警告: 部分语音文件下载失败，但将继续执行")
        
        print("\n上位机语音文件设置已完成!")
        print("\n请注意: 您需要在下位机上运行此程序，以下载对应的动作文件并更新配置")
if __name__ == "__main__":
    sys.exit(main())