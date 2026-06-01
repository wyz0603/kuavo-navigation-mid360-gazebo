# -*- encoding:utf-8 -*-
import shutil
import subprocess
from typing import Iterator
import datetime
import requests
import readline
import json
import time
import binascii
import pyaudio
import wave
import audioop
import time
import os
import sys
import hashlib
import hmac
import base64
from socket import *
import json, time, threading
from websocket import create_connection
import websocket
from urllib.parse import quote
import logging
import numpy as np
import rospy
from std_msgs.msg import Int16MultiArray
from kuavo_audio_player.srv import audio_status
from tts_ws_python3_demo import tts_xunfei

file_format = "pcm"  # 支持 pcm
FORMAT = pyaudio.paInt16
CHANNELS = 1
RATE = 16000
CHUNK = 1024
THRESHOLD = 3000  # 音量阈值
SILENCE_DURATION = 1  # 录音结束后静默持续时间（秒）
RECORD_SECONDS = 60  # 最大录音时间
INPUT_FILE = "input.pcm"

class ROSAudioPublisher:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('voice_chat_bot', anonymous=True)
        
        # 创建音频数据发布器，发布Int16MultiArray格式的音频数据
        self.audio_data_pub = rospy.Publisher('/audio_data', Int16MultiArray, queue_size=10)
        
        # 等待发布器连接
        rospy.sleep(1)
        
        print("ROS语音聊天机器人已启动")

    def publish_pcm_audio_data(self, pcm_file_path, sample_rate=16000, chunk_size=1024):
        """将PCM文件按照int16格式发布到audio_data话题"""
        try:
            print(f"开始发布PCM音频数据: {pcm_file_path}")
            
            # 直接读取PCM文件(假设是16位有符号整数格式)
            with open(pcm_file_path, 'rb') as f:
                pcm_data = f.read()
            
            # 将字节数据转换为int16数组
            audio_array = np.frombuffer(pcm_data, dtype=np.int16)
            print(f"音频长度: {len(audio_array)} 采样点, {len(audio_array)/sample_rate:.2f}秒")
            
            # 分块发送音频数据
            for i in range(0, len(audio_array), chunk_size):
                if rospy.is_shutdown():
                    break
                
                # 获取当前音频块
                audio_chunk = audio_array[i:i+chunk_size]
                
                # 创建音频消息
                msg = Int16MultiArray()
                msg.data = audio_chunk.tolist()
                
                # 发布消息到 /audio_data 话题
                self.audio_data_pub.publish(msg)
                
                # 控制发送频率,保证音频连续播放
                duration = chunk_size / sample_rate
                time.sleep(duration * 0.8)  # 稍微快一点避免断续

        except FileNotFoundError:
            print(f"错误: 找不到文件 {pcm_file_path}")
        except Exception as e:
            print(f"发布失败: {str(e)}")
# 全局发布器实例
audio_publisher = None

def wait_for_audio_completion():
    """等待音频播放完成"""
    try:
        # 等待音频状态服务可用
        rospy.wait_for_service('audio_status', timeout=10)
        audio_status_service = rospy.ServiceProxy('audio_status', audio_status)
        
        # 轮询音频播放状态
        while not rospy.is_shutdown():
            try:
                response = audio_status_service()
                if not response.is_playing:
                    print("未检测到音频输入或音频播放完成，开始下一次识别")
                    break
                else:
                    print("音频正在播放中，等待播放完成...")
                    time.sleep(0.5)  # 每0.5秒检查一次
            except rospy.ServiceException as e:
                print(f"调用音频状态服务失败: {e}")
                break
                
    except rospy.ROSException:
        print("音频状态服务不可用，继续执行")
    except Exception as e:
        print(f"等待音频完成时出错: {e}")

def chat():
    # 初始化PyAudio
    audio = pyaudio.PyAudio()
    # 打开流
    stream = audio.open(format=FORMAT,
                        channels=CHANNELS,
                        rate=RATE,
                        input=True,
                        frames_per_buffer=CHUNK)

    print("请开始提问")

    frames = []
    recording = False
    silence_start = None
    start_time = time.time()

    while True:
        data = stream.read(CHUNK)
        rms = audioop.rms(data, 2)  # 计算音量

        if rms > THRESHOLD:
            if not recording:
                recording = True
                start_time = time.time()
                print("开始录音")
                silence_start = None
            frames.append(data)
            # 更新静默开始时间
            silence_start = None
        else:
            if recording:
                if silence_start is None:
                    silence_start = time.time()
                elif time.time() - silence_start > SILENCE_DURATION:
                    # 结束录音
                    recording = False
                    break

        if time.time() - start_time > RECORD_SECONDS:
            # 达到最大录音时间，自动停止
            break

    # 关闭流
    stream.stop_stream()
    stream.close()
    audio.terminate()

    # 保存录音文件
    with wave.open(INPUT_FILE, 'wb') as wf:
        wf.setnchannels(CHANNELS)
        wf.setsampwidth(audio.get_sample_size(FORMAT))
        wf.setframerate(RATE)
        wf.writeframes(b''.join(frames))

def deepseek_chat(text):
    global audio_publisher
    api_key = "XXXXX"  # 替换为实际DeepSeek API Key
    url = "https://api.deepseek.com/v1/chat/completions"
    
    headers = {
        "Authorization": f"Bearer {api_key}",
        "Content-Type": "application/json"
    }

    payload = {
        "model": "deepseek-chat",  # 使用最新模型
        "messages": [
            {"role": "system", "content": "你叫夸父，是由乐聚机器人研发的功能丰富的人形机器人，回答要简洁专业，控制在60字内"},
            {"role": "user", "content": text}
        ],
        "temperature": 0.2,
        "max_tokens": 1024
    }

    try:
        response = requests.post(url, headers=headers, json=payload)
        response.raise_for_status()  # 自动处理HTTP错误
        reply = response.json()["choices"][0]["message"]["content"]
        
        print("提问：", text)
        print("回答:", reply)
        
        # 调用TTS
        tts_xunfei(reply)
        
        # 将PCM音频数据发布到ROS话题 /audio_data
        if os.path.exists("output.pcm") and audio_publisher:
            audio_publisher.publish_pcm_audio_data("output.pcm")
            # 等待音频播放完成
            wait_for_audio_completion()
        
    except requests.exceptions.HTTPError as err:
        print(f"HTTP错误: {err}")
    except KeyError:
        print("响应解析失败，检查API返回格式")

class Client():
    def __init__(self):
        base_url = "ws://rtasr.xfyun.cn/v1/ws"
        ts = str(int(time.time()))
        tt = (app_id + ts).encode('utf-8')
        md5 = hashlib.md5()
        md5.update(tt)
        baseString = md5.hexdigest()
        baseString = bytes(baseString, encoding='utf-8')

        apiKey = api_key.encode('utf-8')
        signa = hmac.new(apiKey, baseString, hashlib.sha1).digest()
        signa = base64.b64encode(signa)
        signa = str(signa, 'utf-8')
        self.end_tag = "{\"end\": true}"

        self.sentence = None
        self.ws = create_connection(base_url + "?appid=" + app_id + "&ts=" + ts + "&signa=" + quote(signa))
        self.trecv = threading.Thread(target=self.recv)
        self.trecv.start()

    def send(self, file_path):
        file_object = open(file_path, 'rb')
        try:
            index = 1
            while True:
                chunk = file_object.read(1280)
                if not chunk:
                    break
                self.ws.send(chunk)

                index += 1
                time.sleep(0.04)
        finally:
            file_object.close()

        self.ws.send(bytes(self.end_tag.encode('utf-8')))
        print("send end tag success")

    def recv(self):
        try:
            while self.ws.connected:
                result = str(self.ws.recv())
                if len(result) == 0:
                    print("receive result end")
                    break
                result_dict = json.loads(result)
                # 解析结果
                if result_dict["action"] == "started":
                    print("handshake success, result: " + result)

                if result_dict["action"] == "result":
                    result_1 = result_dict
                    data_str = result_1["data"]
                    print("rtasr result: " + result_1["data"])

                if result_dict["action"] == "error":
                    print("rtasr error: " + result)
                    self.ws.close()
                    return
            data_dict = json.loads(data_str)

            words = []
            for rt in data_dict['cn']['st']['rt']:
                for ws in rt['ws']:
                    for cw in ws['cw']:
                        words.append(cw['w'])

            self.sentence = ''.join(words)

        except websocket.WebSocketConnectionClosedException:
            print("receive result end")

    def close(self):
        self.ws.close()
        print("connection closed")


if __name__ == '__main__':
    try:
        logging.basicConfig()
        app_id = "XXXXXXXX"    #替换为rtasr实际的app_id
        api_key = "XXXXXXXXXXXXXXXXXXX"   #替换为rtasr实际的api_key
        
        # 创建ROS音频发布器实例
        audio_publisher = ROSAudioPublisher()
        
        while not rospy.is_shutdown():
            # 在开始录音前，先确保没有音频正在播放
            print("检查音频播放状态...")
            wait_for_audio_completion()
            
            print("开始新一轮语音识别...")
            chat()
            file_path = "input.pcm"
            client = Client()
            client.send(file_path)
            client.trecv.join()  # 确保 recv 线程完成
            # 在主函数中调用 deepseek_chat
            if client.sentence:
                deepseek_chat(client.sentence)
                
    except rospy.ROSInterruptException:
        print("程序被中断")
    except Exception as e:
        print(f"程序运行出错: {e}")