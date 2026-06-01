import sys
import select
import time
from robot_speech import RobotSpeech
import rospy
import traceback

class VoiceConversation:
    def __init__(self):
        rospy.init_node('voice_conversation', anonymous=True)
        self.robot_speech = None
        self.is_running = False
        self.max_retries = 3  # 最大重试次数
        
    def setup_connection(self):
        """建立连接"""
        try:
            self.robot_speech = RobotSpeech()
            print("初始化RobotSpeech成功")
            
            if not self.robot_speech.establish_doubao_speech_connection(
                app_id="xxxxx", 
                access_key="xxxxx"
            ):
                print("语音服务连接失败")
                return False
                
            print("语音服务连接成功")
            return True
            
        except Exception as e:
            print(f"初始化连接失败: {e}")
            return False
    
    def start_interaction(self):
        """启动语音交互"""
        try:
            self.robot_speech.start_speech()
            self.is_running = True
            print("语音交互已启动")
            return True
        except Exception as e:
            print(f"启动语音交互失败: {e}")
            self.is_running = False
            return False
    
    def stop_interaction(self):
        """停止语音交互"""
        if not self.robot_speech or not self.is_running:
            return
            
        try:
            self.robot_speech.stop_speech()
            self.is_running = False
            print("语音交互已停止")
        except Exception as e:
            print(f"停止语音交互时出错: {e}")
    
    def run(self):
        """主运行循环"""
        print("=== 语音交互程序启动 ===")
        
        # 尝试建立连接，允许重试
        for attempt in range(self.max_retries):
            if self.setup_connection():
                break
            elif attempt < self.max_retries - 1:
                print(f"连接失败，{attempt + 1}秒后重试...")
                time.sleep(attempt + 1)
            else:
                print(f"经过{self.max_retries}次尝试后仍连接失败，程序退出")
                return
        
        # 启动语音交互
        if not self.start_interaction():
            print("启动语音交互失败，程序退出")
            return
        
        print("\n指令:")
        print("  c - 停止语音交互")
        print("  r - 重启语音交互")
        print("  q - 退出程序")
        
        try:
            while not rospy.is_shutdown() and self.is_running:
                # 非阻塞检测键盘输入
                if sys.stdin in select.select([sys.stdin], [], [], 0.1)[0]:
                    cmd = sys.stdin.read(1).lower()
                    
                    if cmd == 'c':  # 停止
                        print("停止语音交互")
                        self.stop_interaction()
                        
                    elif cmd == 'r':  # 重启
                        print("重启语音交互...")
                        self.stop_interaction()
                        time.sleep(1)  # 等待清理
                        if not self.start_interaction():
                            print("重启失败")
                            
                    elif cmd == 'q':  # 退出
                        print("退出程序")
                        break
                        
        except KeyboardInterrupt:
            print("\n收到中断信号")
        except Exception as e:
            print(f"主循环运行出错: {e}")
            traceback.print_exc()
        finally:
            # 清理资源
            self.cleanup()
    
    def cleanup(self):
        """清理资源"""
        print("\n正在清理资源...")
        self.stop_interaction()
        print("资源清理完成")
        print("=== 程序结束 ===")

def main():
    # 简单的异常包装
    try:
        app = VoiceConversation()
        app.run()
    except Exception as e:
        print(f"程序发生严重错误: {e}")
        traceback.print_exc()
        print("程序异常退出")

if __name__ == "__main__":
    main()
