#!/usr/bin/env python3

import sys
import os
import json
import pyaudio
import speech_recognition as sr
import configparser
import time
import datetime
import yaml
import rospy
import pygame
from kuavo_msgs.srv import recordmusic, recordmusicResponse

# 获取包的路径
import rospkg
rospack = rospkg.RosPack()
package_path = rospack.get_path('kuavo_audio_recorder')

# 将包的 scripts 目录添加到 Python 模块搜索路径中
scripts_path = os.path.join(package_path, 'scripts')
sys.path.append(scripts_path)

from audio_device import listAudioDevice
from audio_player import AudioPlayer

audio_path = os.path.join(os.environ['HOME'], 'kuavo_ros_application/Music/record_music')

configFile = configparser.ConfigParser()
configFile.read(os.path.join(scripts_path, 'config/config.txt'), encoding='utf-8')
config = dict(configFile.items("config"))

ENERGY_THRESHOLD = int(config["energy_threashold"])
AUDIO_INPUT_DEVICE_INDEX = eval(config["audio_input_device_index"])

audio = pyaudio.PyAudio()
player = AudioPlayer()

def calibrateEnergyThreshold(mic: sr.Microphone):
    rec = sr.Recognizer()
    rec.adjust_for_ambient_noise(mic)
    print("energy_threshold: ", rec.energy_threshold)

def recordVoiceSmart(mic: sr.Microphone, save_file="record.pcm", timeout=10):
    r = sr.Recognizer()
    r.energy_threshold = ENERGY_THRESHOLD

    print("Listening....")

    try:
        audio_data = r.listen(mic, timeout=timeout)
        with open(save_file, "wb") as f:
            f.write(audio_data.get_wav_data())
        print("Record complete.")
        return True  # 表示成功录音
    except sr.WaitTimeoutError:
        print("Listening timed out, no voice detected.")
        return False  # 表示没有检测到语音

def voiceToText(mic: sr.Microphone, save_file, smart=True, timeout=10):
    if smart:
        result = recordVoiceSmart(mic, save_file, timeout)
        # 异常检测
        if not result:
             return " "
        print("smart")

    return True

def recordVoice(music_number, timeout):
    """
        录制音频状态
    """
    global question

    player.playSoundList("speak")
    player.waitForPlayer()

    # 生成保存文件名，包含当前时间戳和自定义名字
    current_date = datetime.datetime.now().strftime("%Y-%m-%d-%H%M")
    save_file = f"{audio_path}/{current_date}_record_{music_number}.pcm"

    with sr.Microphone(AUDIO_INPUT_DEVICE_INDEX, 16000) as mic:
        print("[Log] Audio initialization complete.")

        print("[Audio Device List]")
        listAudioDevice(audio)
        print()

        print("[Measure Ambient Noise]")
        calibrateEnergyThreshold(mic)
        print()

        print("[Start ChatGPT]")         

        # 先清空，然后转语音
        question = " "
        question = voiceToText(mic=mic, save_file=save_file, timeout=timeout)

        if question:
            print("-----录制完毕------")
            return True
        else:
            return False

def record_music_callback(req):
    success = recordVoice(req.music_number, req.time_out)
    response = recordmusicResponse()
    response.success_flag = success
    return response

if __name__ == "__main__":
    rospy.init_node('record_music_node')
    service = rospy.Service('record_music', recordmusic, record_music_callback)
    print("Ready to record music.")
    rospy.spin()
    pygame.quit()
