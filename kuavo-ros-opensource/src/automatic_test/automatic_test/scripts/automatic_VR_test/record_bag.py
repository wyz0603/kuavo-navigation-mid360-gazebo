import os
import rospy
import subprocess
import signal
import time
import datetime
from std_msgs.msg import Bool, String
from rich.console import Console
import questionary
import json
import threading

console = Console()

class RosbagRecorder:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('rosbag_recorder', anonymous=True)
        
        # 创建发布者，用于发布录制状态
        self.recording_pub = rospy.Publisher('/recording_status', Bool, queue_size=10, latch=True)
        self.message_pub = rospy.Publisher('/recording_message', String, queue_size=10, latch=True)
        
        # 订阅录制状态话题，用于控制录制的开始和停止
        self.recording_sub = rospy.Subscriber('/recording_status', Bool, self.recording_callback)   
        
        # 录制进程
        self.record_process = None
        self.is_recording = False
        
        # 加载要录制的话题
        self.topics_to_record = self.load_topics()
        
        # 确保录制目录存在
        self.record_dir = os.path.expanduser("~/rosbag_records")
        if not os.path.exists(self.record_dir):
            os.makedirs(self.record_dir)
        
        # 录制计时器
        self.record_timer = None
        
        # 当前会话名称
        self.current_session_name = None
    
    def load_topics(self):
        """加载要录制的话题列表"""
        try:
            # 尝试从JSON文件加载话题列表
            script_dir = os.path.dirname(os.path.abspath(__file__))
            config_path = os.path.join(script_dir, "record_topics.json")
            
            if not os.path.exists(config_path):
                # 如果配置文件不存在，使用默认话题列表
                default_topics = [
                    "/kuavo_arm_traj",
                    "/sensors_data_raw",
                    "/leju_quest_bone_poses",
                    "/recording_status",
                    "/recording_message",
                    "/quest_joystick_data",
                    "/quest_hand_finger_tf"
                ]
                return default_topics
            
            with open(config_path, 'r') as f:
                config = json.load(f)
            
            # 确保包含录制状态和消息话题
            topics = config.get("record_topics", [])
            if "/recording_status" not in topics:
                topics.append("/recording_status")
            if "/recording_message" not in topics:
                topics.append("/recording_message")
            
            return topics
        except Exception as e:
            console.print(f"[red]加载话题列表失败: {str(e)}，使用默认话题列表[/red]")
            return [
                "/kuavo_arm_traj",
                "/sensors_data_raw",
                "/leju_quest_bone_poses",
                "/recording_status",
                "/recording_message",
                "/quest_joystick_data",
                "/quest_hand_finger_tf"
            ]
    
    def recording_callback(self, msg):
        """处理录制状态消息，根据消息内容开始或停止录制"""
        console.print(f"[blue]收到录制状态消息: {msg.data}[/blue]")
        
        if msg.data and not self.is_recording:
            # 收到True且当前未录制，开始录制
            console.print("[blue]收到录制开始指令[/blue]")
            timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            session_name = f"session_{timestamp}"
            self.start_recording(session_name)
        elif not msg.data and self.is_recording:
            # 收到False且当前正在录制，停止录制
            console.print("[blue]收到录制停止指令[/blue]")
            self.stop_recording()
    
    def start_recording(self, session_name=None):
        """开始录制rosbag"""
        if self.is_recording:
            console.print("[yellow]已经在录制中...[/yellow]")
            return False
        
        try:
            # 生成文件名
            timestamp = time.strftime("%Y%m%d_%H%M%S")
            if session_name:
                filename = f"{session_name}_{timestamp}.bag"
            else:
                filename = f"record_{timestamp}.bag"
            
            filepath = os.path.join(self.record_dir, filename)
            self.current_session_name = session_name
            
            # 构建录制命令
            topics_str = " ".join(self.topics_to_record)
            cmd = f"rosbag record -O {filepath} {topics_str}"
            
            # 启动录制进程
            console.print(f"[blue]开始录制到: {filepath}[/blue]")
            console.print(f"[blue]录制话题: {topics_str}[/blue]")
            console.print(f"[blue]录制时长: 5.5秒[/blue]")

            self.record_process = subprocess.Popen(
                cmd, 
                shell=True, 
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                preexec_fn=os.setsid
            )
            
            self.is_recording = True
            console.print("[green]录制已开始[/green]")
            return True
        
        except Exception as e:
            console.print(f"[red]开始录制失败: {str(e)}[/red]")
            return False
    
    def stop_recording(self):
        """停止录制rosbag"""
        if not self.is_recording or self.record_process is None:
            console.print("[yellow]当前没有录制在进行[/yellow]")
            return False
        
        try:
            # 取消定时器（如果存在）
            if self.record_timer and self.record_timer.is_alive():
                self.record_timer.cancel()
                self.record_timer = None
            
            # 终止录制进程
            os.killpg(os.getpgid(self.record_process.pid), signal.SIGINT)
            
            # 等待进程结束
            self.record_process.wait()
            
            self.is_recording = False
            self.record_process = None
            
            console.print("[green]录制已停止[/green]")
            return True
        
        except Exception as e:
            console.print(f"[red]停止录制失败: {str(e)}[/red]")
            return False

def main():
    recorder = RosbagRecorder()
    
    try:
        # 发布一个录制开始的消息，触发录制
        console.print("[blue]发布录制开始指令...[/blue]")
        recorder.recording_pub.publish(Bool(True))
        
        # 等待5.5秒后自动停止录制
        console.print("[blue]将在5.5秒后自动停止录制...[/blue]")
        time.sleep(9.5)
        
        # 发布录制停止指令
        console.print("[blue]发布录制停止指令...[/blue]")
        recorder.recording_pub.publish(Bool(False))
        
        # 等待录制完成
        while recorder.is_recording and not rospy.is_shutdown():
            rospy.sleep(0.1)
        
        console.print("[green]录制完成，程序退出[/green]")
        
    except KeyboardInterrupt:
        console.print("[yellow]程序被用户中断[/yellow]")
        if recorder.is_recording:
            recorder.recording_pub.publish(Bool(False))
            while recorder.is_recording and not rospy.is_shutdown():
                rospy.sleep(0.1)

if __name__ == "__main__":
    main()
