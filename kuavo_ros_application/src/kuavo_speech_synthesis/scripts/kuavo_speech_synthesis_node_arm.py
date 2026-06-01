#!/opt/lejurobot/kuavo-wifi-announce/venv/bin/python
import rospy
import rospy.service
from std_msgs.msg import Int16MultiArray, MultiArrayDimension
from kuavo_speech_synthesis.srv import SpeechSynthesis, SpeechSynthesisResponse
import numpy as np
import sys
import os
import io
import wave
from scipy.signal import resample
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
import requests
import tarfile
try:
    import sherpa_onnx
except ImportError:
    rospy.logerr("sherpa-onnx 未安装，请安装 sherpa-onnx")
    os.system("pip install sherpa-onnx")    
    os.system("pip install --upgrade soundfile librosa scipy")
import soundfile as sf

current_dir = os.path.dirname(os.path.abspath(__file__))
model_path = os.path.join(current_dir, "../model/sherpa-onnx")
model_url = "https://kuavo.lejurobot.com/statics/kokoro-multi-lang-v1_0.tar.bz2"
class sherpa_onnx_tts:
    def __init__(self):
        # 设置模型路径
        # 获取当前节点的路径
        self.check_model_files(model_path,"kokoro-multi-lang-v1_0",model_url)
        self.sid = rospy.get_param("voice_sid", 52)
        
        # 使用kokoro模型配置
        kokoro_model_path = os.path.join(model_path, "kokoro-multi-lang-v1_0")
        
        tts_config = sherpa_onnx.OfflineTtsConfig(
        model=sherpa_onnx.OfflineTtsModelConfig(
            kokoro=sherpa_onnx.OfflineTtsKokoroModelConfig(
                model=os.path.join(kokoro_model_path, "model.onnx"),
                voices=os.path.join(kokoro_model_path, "voices.bin"),
                tokens=os.path.join(kokoro_model_path, "tokens.txt"),
                data_dir=os.path.join(kokoro_model_path, "espeak-ng-data"),
                dict_dir=os.path.join(kokoro_model_path, "dict"),
                lexicon=os.path.join(kokoro_model_path, "lexicon-us-en.txt") + "," + 
                        os.path.join(kokoro_model_path, "lexicon-zh.txt"),
            ),
            provider="cpu",
            debug=False,
            num_threads=2,
        ),
        max_num_sentences=1,
        )
        self.tts = sherpa_onnx.OfflineTts(tts_config)
    
    def generate_text_to_speech(self, text):
        audio = self.tts.generate(text, sid=self.sid, speed=1.0)
        if len(audio.samples) == 0:
            print("生成音频时出错。请阅读之前的错误消息。")
            return False,None
        audio_duration = len(audio.samples) / audio.sample_rate
        audio.samples = resample(audio.samples, int(audio_duration * 16000))

        wav_buffer = io.BytesIO()
        sf.write(
                wav_buffer,
                audio.samples,
                samplerate=16000,
                format="WAV",
                subtype="PCM_16",
            )
        wav_bytes = wav_buffer.getvalue()
        return True,wav_bytes

    def check_model_files(self,model_path,model_type,model_url):
         # 检查kokoro-multi-lang-v1_0文件夹是否存在
        matcha_dir = os.path.join(model_path, model_type)
        if not os.path.exists(matcha_dir):
            rospy.logerr(f"{model_type}文件夹不存在于路径 {model_path}")
            # 如果模型文件夹不存在，则下载并解压模型文件
            try:
                rospy.loginfo(f"正在下载{model_type}模型文件...")
                response = requests.get(model_url, stream=True)
                if response.status_code == 200:
                    # 创建下载目录
                    os.makedirs(model_path, exist_ok=True)
                    
                    # 下载压缩包
                    tar_path = os.path.join(model_path, "kokoro-multi-lang-v1_0.tar.bz2")
                    with open(tar_path, 'wb') as f:
                        for chunk in response.iter_content(chunk_size=8192):
                            f.write(chunk)
                    
                    rospy.loginfo(f"模型文件下载完成，正在解压到 {model_path}")
                    
                    # 解压文件
                    with tarfile.open(tar_path, "r:bz2") as tar:
                        tar.extractall(path=model_path)
                    
                    # 删除原始压缩包
                    os.remove(tar_path)
                    
                    rospy.loginfo("所有模型文件解压完成")
                    
                    # 再次检查文件夹是否存在
                    if os.path.exists(matcha_dir):
                        rospy.loginfo(f"成功下载并解压kokoro-multi-lang-v1_0模型文件夹: {matcha_dir}")
                    else:
                        rospy.logerr("下载并解压后仍未找到模型文件夹")
                        raise Exception("模型文件夹解压失败")
                else:
                    rospy.logerr(f"下载模型文件失败，HTTP状态码: {response.status_code}")
                    raise Exception(f"下载失败，HTTP状态码: {response.status_code}")
            except Exception as e:
                rospy.logerr(f"下载或解压模型文件时出错: {str(e)}")
                rospy.logerr("请确保已下载并解压kokoro-multi-lang-v1_0模型文件")
                sys.exit(1)
        else:
            rospy.loginfo(f"成功找到kokoro-multi-lang-v1_0模型文件夹: {matcha_dir}")
class SpeechSynthesisNode:
    def __init__(self):
        #self.tts = TTSGenerator()
        rospy.init_node('kuavo_speech_synthesis_node')
        self.service = rospy.Service('speech_synthesis', SpeechSynthesis, self.speech_synthesis_callback)
        self.audio_pub = rospy.Publisher('audio_data', Int16MultiArray, queue_size=10,latch=True)
        self.sherpa_onnx_tts = sherpa_onnx_tts()
        rospy.loginfo("语音合成服务已启动")
        self.rate = rospy.Rate(10)
        self.chunk_size = 8192

    def speech_synthesis_callback(self, req):
        """
        ROS服务回调函数，接收文本并合成语音
    
        Args:
            req: 服务请求，包含要合成的文本
            
        Returns:
            TriggerResponse: 服务响应，包含成功/失败状态和消息
        """
        try:
            text = req.data
            if not text.strip():
                return SpeechSynthesisResponse(success=False)
                
            rospy.loginfo(f"正在合成音频: {text}")
            success,wav = self.sherpa_onnx_tts.generate_text_to_speech(text)
            if success:
                rospy.loginfo("音频合成成功")
                self.publish_audio(wav)    
                return SpeechSynthesisResponse(success=True)
            else:
                rospy.logwarn("音频合成失败")
                return SpeechSynthesisResponse(success=False)
        except Exception as e:
            rospy.logerr(f"语音合成服务发生错误: {e}")
            return SpeechSynthesisResponse(success=False)
    
    def publish_audio(self,wav):
        with io.BytesIO(wav) as wav_file:
            with wave.open(wav_file, 'rb') as wav_file:
                channels = wav_file.getnchannels()
                sample_width = wav_file.getsampwidth()
                frame_rate = wav_file.getframerate()
                n_frames = wav_file.getnframes()
            
                # rospy.loginfo(f"WAV文件信息: 通道数={channels}, 采样宽度={sample_width}, "
                #          f"采样率={frame_rate}, 总帧数={n_frames}")
                # 读取所有音频数据
                audio_data = wav_file.readframes(n_frames)
        if sample_width == 2:  # 16-bit音频
            audio_array = np.frombuffer(audio_data, dtype=np.int16)
        else:
            rospy.logerr(f"不支持的采样宽度: {sample_width}")
            return
          
        for i in range(0, len(audio_array), self.chunk_size):
            if rospy.is_shutdown():
                break
            
            # 获取当前块
            chunk = audio_array[i:i+self.chunk_size]
            
            # 创建消息
            msg = Int16MultiArray()
            msg.data = chunk.tolist()
            
            # 添加元数据
            msg.layout.dim.append(MultiArrayDimension())
            msg.layout.dim[0].label = "audio_samples"
            msg.layout.dim[0].size = len(chunk)
            msg.layout.dim[0].stride = 1

            # 发布消息
            self.audio_pub.publish(msg)
            # rospy.loginfo(f"发布音频块 {i//self.chunk_size + 1}, 大小: {len(chunk)}")
            
            self.rate.sleep()
    def run(self):
        rospy.spin()
def main():
    """
    ROS节点主函数
    """
    speech_synthesis = SpeechSynthesisNode()
    # 保持节点运行
    speech_synthesis.run()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

    
