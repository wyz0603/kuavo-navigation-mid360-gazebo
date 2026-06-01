import os
import sys
import time
from contextlib import contextmanager
from kuavo_audio_player.srv import audio_status
try:    
    import pyaudio
except ImportError:
    print("pyaudio 未安装，先安装 pyaudio")
    command = "sudo apt-get install portaudio19-dev -y && sudo apt-get install python3-pyaudio -y"
    os.system(command)  
    import pyaudio
import rospy
from kuavo_msgs.msg import AudioReceiverData
import numpy as np
try:
    import samplerate
except ImportError:
    print("samplerate 未安装，先安装 samplerate")
    command = "pip install samplerate==0.2.1 -i https://mirrors.aliyun.com/pypi/simple/ --no-input"
    os.system(command)
    import samplerate
import subprocess
import re

@contextmanager
def suppress_alsa_error():
    """
    A context manager to suppress ALSA error messages from underlying C libraries.
    It redirects stderr to /dev/null.
    """
    try:
        devnull = os.open(os.devnull, os.O_WRONLY)
        old_stderr_fileno = os.dup(2)
        sys.stderr.flush()
        os.dup2(devnull, 2)
        os.close(devnull)
        yield
    finally:
        os.dup2(old_stderr_fileno, 2)
        os.close(old_stderr_fileno)

def get_pulseaudio_usb_device_id(target_dev_name: str) -> str:
    """查找PulseAudio中对应USB Composite Device的设备ID"""
    try:
        result = subprocess.check_output(
            ["pactl", "list", "sources"],
            stderr=subprocess.DEVNULL,
            encoding="utf-8"
        )
        pattern = re.compile(r"Name: (alsa_input\..*?)\n.*?device.description = \"(.*?)\"", re.DOTALL)
        devices = pattern.findall(result)
        
        for dev_id, dev_desc in devices:
            if target_dev_name in dev_desc:
                rospy.loginfo(f"找到PulseAudio中USB设备ID: {dev_id} (描述: {dev_desc})")
                return dev_id
        rospy.logwarn("PulseAudio中未找到USB Composite Device设备")
        return ""
    except subprocess.CalledProcessError as e:
        rospy.logerr(f"执行pactl失败: {e}")
        return ""

def release_pulseaudio_device(dev_id: str):
    """修复：使用正确的PulseAudio命令解除设备占用"""
    if not dev_id:
        return
    try:
        # 修复1：暂停设备流（正确命令）
        subprocess.run(
            ["pactl", "suspend-source", dev_id, "1"],
            check=True,
            stderr=subprocess.PIPE,
            stdout=subprocess.PIPE
        )
        # 修复2：移除无效的device.policy设置，改用"独占模式"解锁
        # 备选方案：直接将设备从PulseAudio中卸载（临时）
        subprocess.run(
            ["pactl", "unload-module", "module-alsa-source", "source_name=" + dev_id],
            stderr=subprocess.PIPE,  # 忽略卸载失败（设备可能未加载该模块）
            stdout=subprocess.PIPE
        )
        rospy.loginfo(f"已解除PulseAudio对设备 {dev_id} 的占用")
    except subprocess.CalledProcessError as e:
        rospy.logwarn(f"暂停PulseAudio流失败（非关键）: {e.stderr.decode()}")

def find_usb_audio_device(target_mic_keywords):
    """查找并确保能访问USB Composite Device麦克风"""
    # 步骤1：解除PulseAudio占用
    usb_pa_id = get_pulseaudio_usb_device_id(target_mic_keywords)
    release_pulseaudio_device(usb_pa_id)
    
    # 步骤2：枚举音频设备
    target_device = None
    with suppress_alsa_error():  # 现在能正常执行
        temp_audio = pyaudio.PyAudio()
        try:
            host_api_info = temp_audio.get_host_api_info_by_index(0)
            num_devices = host_api_info.get('deviceCount')
            rospy.loginfo(f"枚举到音频设备总数: {num_devices}")

            for i in range(num_devices):
                try:
                    device_info = temp_audio.get_device_info_by_host_api_device_index(0, i)
                    dev_name = device_info.get('name', '')
                    max_input_ch = device_info.get('maxInputChannels', 0)
                    
                    # 筛选USB Composite Device
                    if max_input_ch > 0 and target_mic_keywords in dev_name:
                        # 校验设备可用性
                        stream = temp_audio.open(
                            format=pyaudio.paInt16,
                            channels=1,
                            rate=int(device_info['defaultSampleRate']),
                            input=True,
                            input_device_index=i,
                            frames_per_buffer=1024,
                            start=False
                        )
                        stream.close()
                        target_device = device_info
                        rospy.loginfo(f"✅ 找到可用USB麦克风: {dev_name} (索引: {i})")
                        break
                    rospy.logdebug(f"跳过设备: {dev_name} (输入声道: {max_input_ch})")
                except Exception as e:
                    rospy.logdebug(f"设备{i}不可用: {e}")
                    continue
        finally:
            temp_audio.terminate()
    
    if not target_device:
        rospy.logerr("❌ 未找到可用的USB Composite Device麦克风！")
    return target_device

class AudioReceiver:
    """
    A class to receive audio from a microphone and publish it to a ROS topic.
    采用短周期开关音频流的方式，避免长时间占用设备。
    """

    def __init__(self, node_name="audio_receiver_node", topic_name="/micphone_data",
                 chunk_size=1024,
                 target_mic_keywords="USB Composite Device",
                 target_sample_rate=16000): 
        """
        Initializes the AudioReceiver.

        Args:
            node_name (str): ROS node name.
            topic_name (str): ROS topic name to publish audio data.
            chunk_size (int): Number of audio frames per buffer.
            target_mic_keywords (list): List of keywords to identify the desired microphone.
            target_sample_rate (int): Target sample rate for output.
        """
        rospy.init_node(node_name)
        self.publisher = rospy.Publisher(topic_name, AudioReceiverData, queue_size=10)
        
        # 设置环境变量，强制使用ALSA后端
        #os.environ['PULSE_RUNTIME_PATH'] = '/dev/null'
        #os.environ['PULSE_STATE_PATH'] = '/dev/null'
        #os.environ['PULSE_CLIENTCONFIG'] = '/dev/null'
        
        self.FORMAT = pyaudio.paInt16
        self.CHUNK = chunk_size

        self.input_device_index = None
        self.mic_channels = None
        self.mic_rate = None
        self.target_mic_keywords = target_mic_keywords
        self.target_sample_rate = target_sample_rate

        rospy.loginfo("开始查找USB麦克风并解除PulseAudio占用...")
        usb_dev = find_usb_audio_device(self.target_mic_keywords)
        if usb_dev:
            rospy.loginfo(f"最终找到的设备信息: {usb_dev}")
        else:
            rospy.logerr("设备查找失败！")

        # 查找设备信息
        self._find_microphone_info()

    def _find_microphone_info(self):
        """
        查找指定麦克风的设备信息
        """
        rospy.loginfo("查找音频输入设备...")
        
        attempt_count = 0
        while not rospy.is_shutdown():
            # 临时创建PyAudio实例来查询设备
            with suppress_alsa_error():
                temp_audio = pyaudio.PyAudio()
                try:
                    info = temp_audio.get_host_api_info_by_index(0)
                    num_devices = info.get('deviceCount')

                    input_devices = []
                    for i in range(0, num_devices):
                        device_info = temp_audio.get_device_info_by_host_api_device_index(0, i)
                        if device_info.get('maxInputChannels') > 0:
                            input_devices.append(device_info)
                finally:
                    temp_audio.terminate()

            if not input_devices:
                rospy.logerr("Error: No audio input devices found.")
                rospy.signal_shutdown("No audio input devices.")
                return

            selected_device_info = None
            for device in input_devices:
                if self.target_mic_keywords in device['name']:
                    self.input_device_index = device['index']
                    selected_device_info = device
                    rospy.loginfo(f"找到目标设备: {device['name']} (Index: {device['index']})")
                    break

            if self.input_device_index is None:
                attempt_count += 1
                if attempt_count >= 3:
                    rospy.logwarn("未找到音频输入设备，已尝试3次，退出节点。")
                    rospy.signal_shutdown("Target microphone not found after 3 attempts.")
                    return
                rospy.sleep(3.0)
            else:
                break

        if selected_device_info:
            self.mic_rate = int(selected_device_info['defaultSampleRate'])
            self.mic_channels = int(selected_device_info['maxInputChannels'])
            rospy.loginfo(f"设备参数: 采样率={self.mic_rate}Hz, 声道数={self.mic_channels}")
        else:
            rospy.logerr("无法确定设备参数，使用默认值。")
            self.mic_rate = 48000
            self.mic_channels = 1

    def _open_stream(self):
        """
        打开音频流
        返回: (audio, stream) 元组
        """
        try:
            with suppress_alsa_error():
                audio = pyaudio.PyAudio()
                stream = audio.open(
                    format=self.FORMAT,
                    channels=self.mic_channels,
                    rate=self.mic_rate,
                    input=True,
                    input_device_index=self.input_device_index,
                    frames_per_buffer=self.CHUNK
                )
            return audio, stream
        except Exception as e:
            rospy.logerr(f"打开音频流失败: {e}")
            return None, None

    def _close_stream(self, audio, stream):
        """
        关闭音频流并释放资源
        """
        try:
            if stream:
                stream.stop_stream()
                stream.close()
            if audio:
                audio.terminate()
        except Exception as e:
            rospy.logerr(f"关闭音频流时出错: {e}")

    def start_listening_and_publishing(self):
        """
        持续采集音频流并发布，采用短周期开关策略
        """
        rospy.loginfo(f"开始持续音频采集 (采样率: {self.mic_rate}Hz → {self.target_sample_rate}Hz)...")
        
        try:
            while not rospy.is_shutdown():
                
                # 打开音频流
                audio, stream = self._open_stream()
                if not audio or not stream:
                    rospy.logwarn("无法打开音频流，等待3秒后重试...")
                    rospy.sleep(3.0)
                    continue
                
                rospy.loginfo("音频流已打开，开始采集...")
                stream_start_time = time.time()
                
                while not rospy.is_shutdown():
                    
                    try:
                        # 读取音频数据
                        data = stream.read(self.CHUNK, exception_on_overflow=False)
                        
                        # 转换为numpy数组
                        audio_np = np.frombuffer(data, dtype=np.int16).astype(np.float32) / 32768.0

                        # 降采样（如果需要）
                        if self.mic_rate != self.target_sample_rate:
                            downsampled_audio_float = samplerate.resample(
                                audio_np, 
                                self.target_sample_rate / self.mic_rate,
                                converter_type='sinc_fastest'
                            )
                            downsampled_audio = np.clip(downsampled_audio_float * 32767, -32768, 32767).astype(np.int16)
                            processed_data = downsampled_audio.tobytes()
                        else:
                            processed_data = data

                        # 发布到ROS话题
                        audio_msg = AudioReceiverData()
                        audio_msg.data = processed_data 
                        self.publisher.publish(audio_msg)
                        # 等待音频播放完成
                        self.wait_for_audio_completion()
                        
                    except Exception as e:
                        rospy.logerr(f"读取音频数据时出错: {e}")
                        break
                
                # 关闭当前流
                self._close_stream(audio, stream)
                rospy.loginfo("音频流已关闭，准备重新打开...")
                
                # 短暂延迟，让设备完全释放
                rospy.sleep(0.1)
                
        except KeyboardInterrupt:
            rospy.loginfo("用户中断程序")
        except Exception as e:
            rospy.logerr(f"运行过程中发生错误: {e}")

    def wait_for_audio_completion(self):
        """等待音频播放完成（通过连续非播放状态判断）"""
        try:
            rospy.wait_for_service('audio_status', timeout=10)
            audio_status_service = rospy.ServiceProxy('audio_status', audio_status)
            
            consecutive_not_playing = 0
            required_consecutive = 10
            check_interval = 0.1
            
            response = audio_status_service()
            if response.is_playing:
                while not rospy.is_shutdown():
                    self.publisher.publish(b'\x00' * 800)
                    try:
                        response = audio_status_service()
                        if not response.is_playing:
                            consecutive_not_playing += 1
                            if consecutive_not_playing >= required_consecutive:
                                break
                        else:
                            consecutive_not_playing = 0
                        
                        time.sleep(check_interval)
                        
                    except rospy.ServiceException as e:
                        rospy.logwarn(f"调用音频状态服务失败: {e}")
                        break
                    
        except rospy.ROSException:
            pass
        except Exception as e:
            rospy.logwarn(f"等待音频完成时出错: {e}")

if __name__ == '__main__':
    try:
        receiver = AudioReceiver(
            node_name="kuavo_audio_receiver",
            target_mic_keywords="USB Composite Device",
            topic_name="/micphone_data",
            chunk_size=1024,
            target_sample_rate=16000
        )
        receiver.start_listening_and_publishing()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Unhandled exception in main: {e}")
