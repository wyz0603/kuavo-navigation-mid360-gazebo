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
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from tts_generate import TTSGenerator

class SpeechSynthesisNode:
    def __init__(self):
        self.tts = TTSGenerator()
        rospy.init_node('kuavo_speech_synthesis_node')
        self.service = rospy.Service('speech_synthesis', SpeechSynthesis, self.speech_synthesis_callback)
        self.audio_pub = rospy.Publisher('audio_data', Int16MultiArray, queue_size=10,latch=True)
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
            success,wav = self.tts.generate_text_to_speech(text, "output.wav")
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

    
