# -*- encoding:utf-8 -*-
import shutil
import subprocess
from typing import Iterator
from typing import *
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
import cv2
from openai import OpenAI
from openai.types.chat.chat_completion import Choice
import re
import requests
import numpy as np
import rospy
from std_msgs.msg import Int16MultiArray
from kuavo_audio_player.srv import audio_status
from std_srvs.srv import Trigger
from rgb_device_select import CameraManager
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

class ROSKimiChatBot:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('kimi_voice_chat_bot', anonymous=True)
        
        # 创建音频数据发布器，发布Int16MultiArray格式的音频数据
        self.audio_data_pub = rospy.Publisher('/audio_data', Int16MultiArray, queue_size=10)
        
        # 等待发布器连接
        rospy.sleep(1)
        
        print("ROS Kimi语音视觉聊天机器人已启动")

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

def wait_for_audio_completion():
    """等待音频播放完成，同时检查缓冲区状态"""
    try:
        # 等待服务可用
        rospy.wait_for_service('audio_status', timeout=10)
        rospy.wait_for_service('get_used_audio_buffer_size', timeout=10)
        
        audio_status_service = rospy.ServiceProxy('audio_status', audio_status)
        buffer_status_service = rospy.ServiceProxy('get_used_audio_buffer_size', Trigger)
        
        # 先检查是否有音频正在发布
        initial_check = True
        consecutive_empty_count = 0
        
        while not rospy.is_shutdown():
            try:
                # 检查音频状态（发布状态+缓冲区状态）
                status_response = audio_status_service()
                
                # 检查缓冲区大小
                buffer_response = buffer_status_service()
                buffer_size = 0
                if buffer_response.success:
                    buffer_size = int(buffer_response.message)
                
                # 如果正在发布或缓冲区有数据，说明还在播放
                is_playing = status_response.is_playing or buffer_size > 0
                
                if not is_playing:
                    consecutive_empty_count += 1
                    # 连续多次检查都是空闲状态，确保播放真正完成
                    if consecutive_empty_count >= 3:
                        if not initial_check:
                            print("没有收音或音频播放完成，开始下一次识别")
                        break
                else:
                    consecutive_empty_count = 0
                    initial_check = False
                    print(f"音频正在播放中(缓冲区:{buffer_size})，等待播放完成...")
                    
                time.sleep(0.3)  # 每0.3秒检查一次
                
            except rospy.ServiceException as e:
                print(f"调用服务失败: {e}")
                break
                
    except rospy.ROSException:
        print("音频服务不可用，继续执行")
    except Exception as e:
        print(f"等待音频完成时出错: {e}")

# 全局发布器实例
ros_bot = None

def recorder():
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

def kimi_vision_chat(text):
    global ros_bot
    for _ in range(80):  # 丢弃80帧确保清空缓冲区
        cap.read()

    ret, frame = cap.read()
    if not ret:
        print("无法获取画面")
        return

    cv2.imwrite('picture.jpg', frame)
    print(f"图片已保存: picture.jpg")

    client = OpenAI(
        api_key="XXXXXXXXXXXXXXXXXXXXXXXXXXX",  # 替换为moonshot实际的api_key
        base_url="https://api.moonshot.cn/v1",
    )
    
    # Kimi 识别的图片地址
    image_path = "picture.jpg"
    
    with open(image_path, "rb") as f:
        image_data = f.read()
    
    # 使用标准库 base64.b64encode 函数将图片编码成 base64 格式的 image_url
    image_url = f"data:image/{os.path.splitext(image_path)[1]};base64,{base64.b64encode(image_data).decode('utf-8')}"
    completion = client.chat.completions.create(
        model="moonshot-v1-8k-vision-preview",
        messages=[
            {"role": "system", "content": "你叫夸父，是由乐聚机器人研发的功能丰富的人形机器人，回答要简洁专业，控制在60字内,我输出给你的是你摄像头看到的画面，你可以回答我看到了......。"},
            {
                "role": "user",
                "content": [
                    {
                        "type": "image_url",  # 使用 image_url 类型来上传图片，内容为使用 base64 编码过的图片内容
                        "image_url": {
                            "url": image_url,
                        },
                    },
                    {
                        "type": "text",
                        "text": text,  # 使用 text 类型来提供文字指令
                    },
                ],
            },
        ],
    )
    
    print("提问：", text)
    print("回答：", completion.choices[0].message.content)

    # 调用TTS
    tts_xunfei(completion.choices[0].message.content)
    
    # 将PCM音频数据发布到ROS话题 /audio_data
    if os.path.exists("output.pcm") and ros_bot:
        ros_bot.publish_pcm_audio_data("output.pcm")
        # 短暂等待确保音频数据进入缓冲区
        time.sleep(0.5)
        # 等待音频播放完成
        wait_for_audio_completion()

def search_impl(arguments: Dict[str, Any]) -> Any:
    return arguments

def chat(messages) -> Choice:
    client = OpenAI(
        base_url="https://api.moonshot.cn/v1",
        api_key="XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX",  # 替换为moonshot实际的api_key
    )
    completion = client.chat.completions.create(
        model="moonshot-v1-128k",
        messages=messages,
        temperature=0.3,
        tools=[
            {
                "type": "builtin_function",  # 使用 builtin_function 声明 $web_search 函数，请在每次请求都完整地带上 tools 声明
                "function": {
                    "name": "$web_search",
                },
            }
        ]
    )
    return completion.choices[0]

def kimi_network_chat(text):
    global ros_bot
    
    messages = [
        {"role": "system", "content": "你叫夸父，是由乐聚机器人研发的功能丰富的人形机器人，回答要简洁专业，控制在60字内."},
    ]

    # 初始提问
    messages.append({
        "role": "user",
        "content": text
    })

    finish_reason = None
    while finish_reason is None or finish_reason == "tool_calls":
        choice = chat(messages)
        finish_reason = choice.finish_reason
        if finish_reason == "tool_calls":  # 判断当前返回内容是否包含 tool_calls
            messages.append(choice.message)  # 将 Kimi 大模型返回给我们的 assistant 消息也添加到上下文中，以便于下次请求时 Kimi 大模型能理解我们的诉求
            for tool_call in choice.message.tool_calls:  # tool_calls 可能是多个，因此我们使用循环逐个执行
                tool_call_name = tool_call.function.name
                tool_call_arguments = json.loads(tool_call.function.arguments)  # arguments 是序列化后的 JSON Object，需要使用 json.loads 反序列化一下
                if tool_call_name == "$web_search":
                    tool_result = search_impl(tool_call_arguments)
                else:
                    tool_result = f"Error: unable to find tool by name '{tool_call_name}'"

                messages.append({
                    "role": "tool",
                    "tool_call_id": tool_call.id,
                    "name": tool_call_name,
                    "content": json.dumps(tool_result),  # 约定使用字符串格式向 Kimi 大模型提交工具调用结果，因此在这里使用 json.dumps 将执行结果序列化成字符串
                })

    print("提问：", text)
    print("回答：", choice.message.content)

    # 调用TTS
    tts_xunfei(choice.message.content)
    
    # 将PCM音频数据发布到ROS话题 /audio_data
    if os.path.exists("output.pcm") and ros_bot:
        ros_bot.publish_pcm_audio_data("output.pcm")
        # 短暂等待确保音频数据进入缓冲区
        time.sleep(0.5)
        # 等待音频播放完成
        wait_for_audio_completion()

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
            print(self.sentence)

        except websocket.WebSocketConnectionClosedException:
            print("receive result end")

    def close(self):
        self.ws.close()
        print("connection closed")


if __name__ == '__main__':
    try:
        logging.basicConfig()
        app_id = "XXXXXXXX"    #替换为rtasr实际的app_id
        api_key = "XXXXXXXXXXXXXXXXXXXXX"   #替换为rtasr实际的api_key
        
        # 创建ROS Kimi聊天机器人实例
        ros_bot = ROSKimiChatBot()
        
        cam_manager = CameraManager()
        rgb_index = cam_manager.get_rgb_camera_index()
        cap = cv2.VideoCapture(rgb_index)  # 替换为实际的摄像头设备索引
        
        while not rospy.is_shutdown():
            # 在开始录音前，先确保没有音频正在播放
            print("检查音频播放状态...")
            wait_for_audio_completion()
            
            print("开始新一轮语音识别...")
            recorder()
            file_path = "input.pcm"
            client = Client()
            client.send(file_path)
            client.trecv.join()  # 确保 recv 线程完成
            pattern = r"联网|查询|搜"  # 联网搜索关键词
            # 在主函数中调用 kimi_network_chat，kimi_vision_chat
            if client.sentence:
                if re.search(pattern, client.sentence):
                    kimi_network_chat(client.sentence)
                else:
                    kimi_vision_chat(client.sentence)
                    
    except rospy.ROSInterruptException:
        print("ROS节点被中断")
    except KeyboardInterrupt:
        print("\n检测到Ctrl+C，正在清理资源...")
    finally:
        # 无论是否发生异常，都执行清理操作
        if 'cap' in locals():
            cap.release()
        cv2.destroyAllWindows()
        print("资源已释放，程序退出")