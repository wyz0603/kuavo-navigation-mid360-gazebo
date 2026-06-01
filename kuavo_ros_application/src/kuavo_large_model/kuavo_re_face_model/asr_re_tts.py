import pyaudio
import wave
import audioop
import time
import os
import hashlib
import hmac
import base64
import json
import threading
from websocket import create_connection
import websocket
from urllib.parse import quote, urlencode
import logging
import numpy as np
import rospy
from std_msgs.msg import Int16MultiArray
from kuavo_audio_player.srv import audio_status
from std_srvs.srv import Trigger
from face_detect import FaceDetect
import ssl
from wsgiref.handlers import format_date_time
from datetime import datetime
from time import mktime
import _thread as thread
import signal
import sys

# 音频参数设置
FORMAT = pyaudio.paInt16
CHANNELS = 1
RATE = 16000
CHUNK = 1024
THRESHOLD = 3000  # 音量阈值
SILENCE_DURATION = 1  # 录音结束后静默持续时间（秒）
RECORD_SECONDS = 60  # 最大录音时间
INPUT_FILE = "input.pcm"

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

class ROSAudioPublisher:
    def __init__(self):
      
        
        # 创建音频数据发布器，发布Int16MultiArray格式的音频数据
        self.audio_data_pub = rospy.Publisher('/audio_data', Int16MultiArray, queue_size=10)
        
        # 等待发布器连接
        rospy.sleep(1)
        
        print("ROS人脸识别语音机器人已启动")

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

class Ws_Param(object):
    def __init__(self, APPID, APIKey, APISecret, Text):
        self.APPID = APPID
        self.APIKey = APIKey
        self.APISecret = APISecret
        self.Text = Text

        self.CommonArgs = {"app_id": self.APPID}
        # 修改为输出PCM格式
        self.BusinessArgs = {"aue": "raw", "auf": "audio/L16;rate=16000", "vcn": "xiaoyan", "tte": "utf8"}
        self.Data = {"status": 2, "text": str(base64.b64encode(self.Text.encode('utf-8')), "UTF8")}

    def create_url(self):
        url = 'wss://tts-api.xfyun.cn/v2/tts'
        now = datetime.now()
        date = format_date_time(mktime(now.timetuple()))

        signature_origin = "host: " + "ws-api.xfyun.cn" + "\n"
        signature_origin += "date: " + date + "\n"
        signature_origin += "GET " + "/v2/tts " + "HTTP/1.1"

        signature_sha = hmac.new(self.APISecret.encode('utf-8'), signature_origin.encode('utf-8'),
                               digestmod=hashlib.sha256).digest()
        signature_sha = base64.b64encode(signature_sha).decode(encoding='utf-8')

        authorization_origin = f'api_key="{self.APIKey}", algorithm="hmac-sha256", headers="host date request-line", signature="{signature_sha}"'
        authorization = base64.b64encode(authorization_origin.encode('utf-8')).decode(encoding='utf-8')

        v = {
            "authorization": authorization,
            "date": date,
            "host": "ws-api.xfyun.cn"
        }
        url = url + '?' + urlencode(v)
        return url

def on_message(ws, message):
    try:
        message = json.loads(message)
        code = message["code"]
        sid = message["sid"]
        audio = message["data"]["audio"]
        audio = base64.b64decode(audio)
        status = message["data"]["status"]
        
        if status == 2:
            print("ws is closed")
            ws.close()
        if code != 0:
            errMsg = message["message"]
            print("sid:%s call error:%s code is:%s" % (sid, errMsg, code))
        else:
            # 修改为保存PCM格式
            with open('./output.pcm', 'ab') as f:
                f.write(audio)

    except Exception as e:
        print("receive msg,but parse exception:", e)

def on_error(ws, error):
    print("### error:", error)

def on_close(ws, one, two):
    print("### closed ###")

def on_open(ws):
    def run(*args):
        d = {"common": wsParam.CommonArgs,
             "business": wsParam.BusinessArgs,
             "data": wsParam.Data,
             }
        d = json.dumps(d)
        print("------>开始发送文本数据")
        ws.send(d)
        # 修改为删除PCM文件
        if os.path.exists('./output.pcm'):
            os.remove('./output.pcm')

    thread.start_new_thread(run, ())

def tts_xunfei(text):
    global wsParam
    wsParam = Ws_Param(APPID='XXXXXXXX', APISecret='XXXXXXXXXXXXXXXXXXXXXXXXXX',
                      APIKey='XXXXXXXXXXXXXXXXXXXXXXXXXXXX',
                      Text=text)#替换为tts实际的app_id,APISecret,APIKey
    websocket.enableTrace(False)
    wsUrl = wsParam.create_url()
    ws = websocket.WebSocketApp(wsUrl, on_message=on_message, on_error=on_error, on_close=on_close)
    ws.on_open = on_open
    ws.run_forever(sslopt={"cert_reqs": ssl.CERT_NONE})

class ASRClient:
    def __init__(self, ros_audio_pub):
        self.app_id = "XXXXXXXX"    # 替换为rtasr实际的app_id
        self.api_key = "XXXXXXXXXXXXXXXXXXXXXXX"   # 替换为rtasr实际的api_key
        self.face_detector = FaceDetect()
        self.running = True
        self.ros_audio_pub = ros_audio_pub
        signal.signal(signal.SIGINT, self.signal_handler)
        
    def signal_handler(self, signum, frame):
        print("\n正在停止程序...")
        self.running = False
        
    def chat(self):
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

        try:
            while self.running:
                data = stream.read(CHUNK, exception_on_overflow=False)
                rms = audioop.rms(data, 2)  # 计算音量

                if rms > THRESHOLD:
                    if not recording:
                        recording = True
                        start_time = time.time()
                        print("开始录音")
                        silence_start = None
                    frames.append(data)
                    silence_start = None
                else:
                    if recording:
                        if silence_start is None:
                            silence_start = time.time()
                        elif time.time() - silence_start > SILENCE_DURATION:
                            recording = False
                            break

                if time.time() - start_time > RECORD_SECONDS:
                    break

        except KeyboardInterrupt:
            print("\n检测到键盘中断，正在停止...")
        finally:
            # 关闭流
            stream.stop_stream()
            stream.close()
            audio.terminate()

            # 只有在有录音数据时才保存文件
            if frames:
                with wave.open(INPUT_FILE, 'wb') as wf:
                    wf.setnchannels(CHANNELS)
                    wf.setsampwidth(audio.get_sample_size(FORMAT))
                    wf.setframerate(RATE)
                    wf.writeframes(b''.join(frames))

    def process_speech(self, text):
        """处理语音识别结果"""
        if "是谁" in text:
            print("检测到'是谁'关键词，开始人脸识别...")
            # 调用人脸识别
            self.face_detector.process_image()
            # 等待人脸识别结果
            rospy.sleep(1)  # 给一些时间让人脸识别完成
            
            # 获取识别到的人脸列表
            detected_faces = self.face_detector.detected_faces
            
            if detected_faces:
                for i, face_name in enumerate(detected_faces):
                    tts_xunfei(f"{face_name}你好")
                    
                    # 将PCM音频数据发布到ROS话题 /audio_data
                    if os.path.exists("output.pcm"):
                        self.ros_audio_pub.publish_pcm_audio_data("output.pcm")
                        # 短暂等待确保音频数据进入缓冲区
                        time.sleep(0.5)
                        # 等待音频播放完成
                        wait_for_audio_completion()
            else:
                tts_xunfei(f"不好意思，我不认识你")
                
                # 将PCM音频数据发布到ROS话题 /audio_data
                if os.path.exists("output.pcm"):
                    self.ros_audio_pub.publish_pcm_audio_data("output.pcm")
                    # 短暂等待确保音频数据进入缓冲区
                    time.sleep(0.5)
                    # 等待音频播放完成
                    wait_for_audio_completion()

        else:
            print("未检测到'是谁'关键词")

    def start_recognition(self):
        base_url = "ws://rtasr.xfyun.cn/v1/ws"
        ts = str(int(time.time()))
        tt = (self.app_id + ts).encode('utf-8')
        md5 = hashlib.md5()
        md5.update(tt)
        baseString = md5.hexdigest()
        baseString = bytes(baseString, encoding='utf-8')

        apiKey = self.api_key.encode('utf-8')
        signa = hmac.new(apiKey, baseString, hashlib.sha1).digest()
        signa = base64.b64encode(signa)
        signa = str(signa, 'utf-8')
        self.end_tag = "{\"end\": true}"

        self.sentence = None
        self.ws = create_connection(base_url + "?appid=" + self.app_id + "&ts=" + ts + "&signa=" + quote(signa))
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
            if self.sentence:
                self.process_speech(self.sentence)

        except websocket.WebSocketConnectionClosedException:
            print("receive result end")

    def close(self):
        self.ws.close()
        print("connection closed")

def main():
    try:
        asr_client = ASRClient(None)  # 先创建ASRClient，让FaceDetect初始化ROS节点
        
        # 然后创建ROS音频发布器实例
        ros_audio_pub = ROSAudioPublisher()
        
        # 将音频发布器赋值给ASRClient
        asr_client.ros_audio_pub = ros_audio_pub
        
        while not rospy.is_shutdown() and asr_client.running:
            # 在开始录音前，先确保没有音频正在播放
            print("检查音频播放状态...")
            wait_for_audio_completion()
            
            print("开始新一轮语音识别...")
            asr_client.chat()
            if not asr_client.running:
                break
            file_path = "input.pcm"
            asr_client.start_recognition()
            asr_client.send(file_path)
            asr_client.trecv.join()
            asr_client.close()
            
    except rospy.ROSInterruptException:
        print("ROS节点被中断")
    except KeyboardInterrupt:
        print("\n程序正在退出...")
    finally:
        print("程序已退出")

if __name__ == '__main__':
    main()
