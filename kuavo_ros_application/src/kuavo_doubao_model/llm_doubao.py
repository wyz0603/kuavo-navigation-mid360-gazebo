import numpy as np
import rospy
#from kuavo_humanoid_sdk.common.logger import SDKLogger
from std_msgs.msg import Int16MultiArray
from lib.microphone import Microphone
from lib.audio_manager import DialogSession
from lib.realtime_dialog_client import RealtimeDialogClient
from lib import config
import os
import asyncio
import threading
import queue
import time
import struct
import uuid
from typing import Optional, Dict, Any
from queue import Queue
import traceback
from action_controllers import ActionController
from kuavo_msgs.srv import getCurrentGaitName, getCurrentGaitNameRequest
from rospy import ServiceException
from kuavo_msgs.srv import changeArmCtrlMode, changeArmCtrlModeRequest


class ROSDialogSession(DialogSession):
    """Custom DialogSession that integrates with ROS audio system"""
    
    def __init__(self, ws_config: Dict[str, Any], enable_signal_handler: bool = False):
        # Initialize session ID and client without calling parent __init__
        self.session_id = str(uuid.uuid4())
        self.client = RealtimeDialogClient(config=ws_config, session_id=self.session_id)
        self.audio_data_publisher = rospy.Publisher('audio_data', Int16MultiArray, queue_size=10)
        
        # Initialize session state variables
        self.is_running = True
        self.is_session_finished = False
        self.is_user_querying = False
        self.is_sending_chat_tts_text = False
        self.audio_buffer = b''
        
        # Audio chunk buffer for handling large audio chunks
        self.audio_chunk_buffer = []
        self.buffer_lock = threading.Lock()
        
        # Skip PyAudio initialization completely
        self.audio_device = None
        self.audio_queue = None
        self.output_stream = None
        self.input_stream = None
        self.player_thread = None
        self.is_recording = True
        self.is_playing = False  # We don't use PyAudio playing

        self.action_mode = True
        self.action_controller = ActionController()
        
        # Start audio chunk processing thread
        self.chunk_processor_thread = threading.Thread(target=self._process_audio_chunks)
        self.chunk_processor_thread.daemon = True
        self.chunk_processor_thread.start()
        
        # Only set signal handler if requested and in main thread
        if enable_signal_handler:
            try:
                import signal
                signal.signal(signal.SIGINT, self._keyboard_signal)
            except ValueError as e:
                print(f"Warning: Cannot set signal handler (not in main thread): {e}")

    def cleanup(self):
        """Clean up resources (no PyAudio to clean up)"""
        self.is_running = False
        self.is_recording = False
        self.is_playing = False
        
        # Clear audio buffer
        with self.buffer_lock:
            self.audio_chunk_buffer.clear()
        
        # Wait for chunk processor thread to finish
        if hasattr(self, 'chunk_processor_thread') and self.chunk_processor_thread.is_alive():
            self.chunk_processor_thread.join(timeout=2)
        
        # No audio device cleanup needed since we're using ROS

    def _keyboard_signal(self, sig, frame):
        """Handle keyboard interrupt signal"""
        print(f"receive keyboard Ctrl+C")
        self.is_recording = False
        self.is_playing = False
        self.is_running = False

    def _process_audio_chunks(self):
        """Process buffered audio chunks in a separate thread"""
        while self.is_running:
            try:
                with self.buffer_lock:
                    if self.audio_chunk_buffer:
                        chunk = self.audio_chunk_buffer.pop(0)
                    else:
                        chunk = None
                
                if chunk:
                    self._publish_audio_chunk_to_ros(chunk)
                    # Small delay to prevent overwhelming the ROS system
                    time.sleep(0.01)
                else:
                    # No chunks to process, wait a bit
                    time.sleep(0.05)
                    
            except Exception as e:
                print(f"[Speech] Error processing audio chunks: {e}")
                time.sleep(0.1)

    def _add_audio_chunk_to_buffer(self, audio_chunk):
        """Add audio chunk to buffer for processing"""
        with self.buffer_lock:
            self.audio_chunk_buffer.append(audio_chunk)
            # Limit buffer size to prevent memory issues
            if len(self.audio_chunk_buffer) > 50:
                print(f"[Speech] Audio buffer full, dropping oldest chunk")
                self.audio_chunk_buffer.pop(0)

    def _convert_audio_bytes_to_int_list(self, audio_bytes: bytes):
        """Convert audio bytes (PCM) to list of integers for ROS audio playback"""
        try:
            # print(f"[Speech] Converting audio bytes: length={len(audio_bytes)}")
            
            if len(audio_bytes) < 4:  # Float32 needs at least 4 bytes
                print(f"[Speech] Audio data too short: {len(audio_bytes)} bytes")
                return []
            
            # Try to detect audio format and convert accordingly
            audio_ints = self._convert_audio_with_format_detection(audio_bytes)
            
            if audio_ints:
                # Resample from 24kHz to 16kHz for ROS compatibility
                audio_ints = self._resample_audio(audio_ints, 24000, 16000)
                
                # Split large audio chunks into smaller ones for better ROS compatibility
                chunk_size = 8192
                if len(audio_ints) > chunk_size:
                    print(f"[Speech] Splitting large audio chunk ({len(audio_ints)} samples) into {len(audio_ints) // chunk_size + 1} smaller chunks")
                    # Split into multiple chunks and add to buffer
                    for i in range(0, len(audio_ints), chunk_size):
                        chunk = audio_ints[i:i + chunk_size]
                        if len(chunk) > 0:
                            self._add_audio_chunk_to_buffer(chunk)
                    return []  # Return empty since we've buffered the chunks
                else:
                    return audio_ints
            else:
                print(f"[Speech] No audio samples extracted from {len(audio_bytes)} bytes")
                return []
            
        except Exception as e:
            print(f"[Speech] Error converting audio bytes to int list: {e}")
            return []

    def _convert_audio_with_format_detection(self, audio_bytes: bytes):
        """Convert audio bytes with automatic format detection"""
        import struct
        import numpy as np
        
        # Try Float32 format first (24kHz server format)
        try:
            if len(audio_bytes) % 4 == 0:  # Float32 should be divisible by 4
                float_samples = []
                for i in range(0, len(audio_bytes), 4):
                    if i + 3 < len(audio_bytes):
                        # Convert 4 bytes to float32 (little-endian)
                        sample_float = struct.unpack('<f', audio_bytes[i:i+4])[0]
                        float_samples.append(sample_float)
                
                if float_samples:
                    # Analyze float32 data range
                    min_float = min(float_samples)
                    max_float = max(float_samples)
                    abs_max = max(abs(min_float), abs(max_float))
                    
                    # print(f"[Speech] Float32 analysis: count={len(float_samples)}, min={min_float:.6f}, max={max_float:.6f}, abs_max={abs_max:.6f}")
                    
                    # Auto-detect gain based on actual float range
                    if abs_max > 0.001:  # Avoid division by zero
                        # Calculate gain to use full int16 range
                        # Leave some headroom (use 0.9 instead of 1.0)
                        target_range = 32767 * 0.9
                        auto_gain = target_range / abs_max
                        
                        # print(f"[Speech] Auto-detected gain: {auto_gain:.2f}")
                        
                        # Apply gain and convert to int16
                        samples = []
                        for sample_float in float_samples:
                            sample_int = int(sample_float * auto_gain)
                            sample_int = max(-32768, min(32767, sample_int))  # Clamp
                            samples.append(sample_int)
                        
                        # Check final result
                        min_val = min(samples)
                        max_val = max(samples)
                        variation = max_val - min_val
                        
                        # print(f"[Speech] Float32 conversion result: count={len(samples)}, variation={variation}")
                        
                        if variation > 100:  # Good variation suggests valid conversion
                            # print(f"[Speech] Using Float32 format (24kHz) with auto-gain {auto_gain:.2f}")
                            return samples
                    else:
                        print(f"[Speech] Float32 data range too small (abs_max={abs_max:.6f})")
        except Exception as e:
            print(f"[Speech] Float32 conversion failed: {e}")
        
        # Try Int16 format (fallback)
        try:
            if len(audio_bytes) % 2 == 0:  # Int16 should be divisible by 2
                samples = []
                for i in range(0, len(audio_bytes), 2):
                    if i + 1 < len(audio_bytes):
                        # Convert 2 bytes to signed 16-bit integer (little-endian)
                        sample = struct.unpack('<h', audio_bytes[i:i+2])[0]
                        samples.append(sample)
                
                if samples:
                    min_val = min(samples)
                    max_val = max(samples)
                    variation = max_val - min_val
                    
                    print(f"[Speech] Int16 conversion: count={len(samples)}, variation={variation}")
                    print(f"[Speech] Using Int16 format")
                    return samples
        except Exception as e:
            print(f"[Speech] Int16 conversion failed: {e}")
        
        print(f"[Speech] Failed to convert audio data with any format")
        return []

    def _resample_audio(self, audio_samples, from_rate, to_rate):
        """Resample audio from one sample rate to another"""
        if from_rate == to_rate:
            return audio_samples
        
        try:
            import numpy as np
            from scipy import signal
            
            # Convert to numpy array
            audio_array = np.array(audio_samples, dtype=np.float32)
            
            # Calculate resampling ratio
            resample_ratio = to_rate / from_rate
            
            # Resample using scipy
            resampled_length = int(len(audio_array) * resample_ratio)
            resampled_audio = signal.resample(audio_array, resampled_length)
            
            # Convert back to int16 and clamp
            resampled_int = np.clip(resampled_audio, -32768, 32767).astype(np.int16)
            
            # print(f"[Speech] Resampled audio from {from_rate}Hz to {to_rate}Hz: {len(audio_samples)} -> {len(resampled_int)} samples")
            
            return resampled_int.tolist()
            
        except ImportError:
            print(f"[Speech] scipy not available, using simple decimation for resampling")
            # Simple decimation fallback
            if from_rate > to_rate:
                step = int(from_rate // to_rate)
                return audio_samples[::step]
            else:
                return audio_samples
        except Exception as e:
            print(f"[Speech] Error resampling audio: {e}")
            return audio_samples

    def _publish_audio_chunk_to_ros(self, audio_int_list, gain: int = 1):
        """Publish single audio chunk directly to ROS topic using Audio interface"""
        try:
            if not audio_int_list:
                return
                
            # Use the new publish_audio_chunk method from Audio class
            success = self.publish_audio_chunk(audio_int_list, gain=gain)
            
            if not success:
                print(f"[Speech] Failed to publish audio chunk with {len(audio_int_list)} samples")
            
        except Exception as e:
            print(f"[Speech] Error publishing audio to ROS: {e}")

    def handle_server_response(self, response: Dict[str, Any]) -> None:
        """Override to handle audio playback through ROS instead of PyAudio"""
        if response == {}:
            return
            
        # Handle audio data from server
        if response['message_type'] == 'SERVER_ACK' and isinstance(response.get('payload_msg'), bytes):
            if self.is_sending_chat_tts_text:
                return
                
            audio_data = response['payload_msg']
            self.audio_buffer += audio_data
            
            # print(f"[Speech] Received audio chunk: {len(audio_data)} bytes")
            
            # Play audio through ROS audio system instead of PyAudio
            try:
                audio_int_list = self._convert_audio_bytes_to_int_list(audio_data)
                if audio_int_list:
                    # For smaller chunks, publish immediately
                    self._publish_audio_chunk_to_ros(audio_int_list)
                # For larger chunks, they are automatically buffered in _convert_audio_bytes_to_int_list
            except Exception as e:
                print(f"[Speech] Error playing server audio through ROS: {e}")
                
        elif response['message_type'] == 'SERVER_FULL_RESPONSE':
            # print(f"服务器响应: {response}")
            event = response.get('event')
            payload_msg = response.get('payload_msg', {})

            # Log ASR results (user speech recognition)
            if event == 451:
                # Extract user speech text from ASR results
                results = payload_msg.get('results', [])
                if results and len(results) > 0:
                    result = results[0]
                    text = result.get('text', '')
                    is_interim = result.get('is_interim', True)
                    
                    # Only log final results (not interim)
                    if not is_interim and text:
                        print(f"[Speech] 用户说话: {text}")
                         # 先检查步态
                        is_standing = self.check_gait()
                    
                        # 再检查手臂控制模式
                        arm_mode = self.get_current_arm_ctrl_mode()


                        if is_standing and arm_mode == 1:
                            print(f"[Action] 条件满足：站立={is_standing}, 手臂模式={arm_mode}")
                            self.action_mode = self.action_controller.handle(text, self.action_mode)
                        else:
                            print(f"[Action] 条件不满足：站立={is_standing}, 手臂模式={arm_mode}")


            # Log TTS streaming text (AI response)
            elif event == 550:
                content = payload_msg.get('content', '')
                if content:
                    # Use info level for visible logging, accumulate content for complete response
                    if not hasattr(self, '_current_ai_response'):
                        self._current_ai_response = ""
                    self._current_ai_response += content
                    # print(f"[Speech] AI回复: {content}")

            if event == 450:
                print(f"清空缓存音频: {response['session_id']}")
                # Clear the audio buffer
                with self.buffer_lock:
                    self.audio_chunk_buffer.clear()
                self.is_user_querying = True

            if event == 350 and self.is_sending_chat_tts_text and payload_msg.get("tts_type") == "chat_tts_text":
                # Clear the audio buffer
                with self.buffer_lock:
                    self.audio_chunk_buffer.clear()
                self.is_sending_chat_tts_text = False

            if event == 459:
                self.is_user_querying = False
                
            # Log complete AI response when TTS ends
            if event == 351:
                # TTS synthesis completed
                if hasattr(self, '_current_ai_response') and self._current_ai_response:
                    print(f"[Speech] AI完整回复: {self._current_ai_response}")
                    self._current_ai_response = ""  # Reset for next response
                
        elif response['message_type'] == 'SERVER_ERROR':
            print(f"服务器错误: {response['payload_msg']}")
            raise Exception("服务器错误")

    async def receive_loop(self):
        """接收服务器响应的循环"""
        try:
            while True:
                response = await self.client.receive_server_response()
                self.handle_server_response(response)
                if 'event' in response and (response['event'] == 152 or response['event'] == 153):
                    print(f"receive session finished event: {response['event']}")
                    self.is_session_finished = True
                    break
        except asyncio.CancelledError:
            print("接收任务已取消")
        except Exception as e:
            print(f"接收消息错误: {e}")
        finally:
            self.stop()
            self.is_session_finished = True

    def stop(self):
        self.is_recording = False
        self.is_playing = False
        self.is_running = False
            
    def publish_audio_chunk(self, audio_chunk, gain: int = 1):
        """Publish a single audio chunk to the topic, for real-time audio streaming"""
        try:
            if not audio_chunk:
                return False
                
            # 应用增益
            amplified_chunk = [int(sample * gain) for sample in audio_chunk]
            
            # 创建并发布消息
            msg = Int16MultiArray()
            msg.data = amplified_chunk
            
            self.audio_data_publisher.publish(msg)
            # print(f"[Robot Audio] 发布音频块，大小: {len(amplified_chunk)}")
            
            return True
            
        except Exception as e:
            print(f"[Robot Audio] 发布音频块时出错: {e}")
            return False
        
    def check_gait(self):
        try:
            rospy.wait_for_service('/humanoid_get_current_gait_name', timeout=5)
            get_gait = rospy.ServiceProxy('/humanoid_get_current_gait_name', getCurrentGaitName)
            response = get_gait()
            print(f"Success: {response.success}")
            print(f"Current gait: {response.gait_name}")
            
            # 检查是否是stance
            if response.gait_name == "stance":
                print("机器人当前处于站立姿态")
                return True
            else:
                print(f"机器人当前姿态: {response.gait_name}")
                return False
        except ServiceException as e:
            print(f"服务调用失败: {e}")
            return False
        
    def get_current_arm_ctrl_mode(self):
        """
        获取当前手臂控制模式
        注意：传入的control_mode参数会被忽略，始终返回当前实际模式
        
        Returns:
            int: 当前控制模式 (0, 1, 2)，调用失败时返回 -1
        """
        try:
            # 等待服务可用
            rospy.wait_for_service('/humanoid_get_arm_ctrl_mode', timeout=2.0)
            
            # 创建服务代理
            get_mode_proxy = rospy.ServiceProxy('/humanoid_get_arm_ctrl_mode', changeArmCtrlMode)
            
            # 创建请求 - 传入任意值都可以（0,1,2都行）
            req = changeArmCtrlModeRequest()
            req.control_mode = 0  # 任意值，会被忽略
            
            # 调用服务
            resp = get_mode_proxy(req)
            
            if resp.result:
                rospy.loginfo(f"当前手臂控制模式为: {resp.mode}")
                return resp.mode
            else:
                rospy.logwarn(f"查询失败: {resp.message}")
                return -1
                
        except (rospy.ServiceException, rospy.ROSException) as e:
            rospy.logerr(f"获取手臂控制模式失败: {e}")
            return -1


class RobotLLMDoubaoCore:

    def __init__(self, subscribe_topic: str = "/micphone_data"):
        # Microphone interface
        self.microphone = Microphone(subscribe_topic)
        
        # ROS Audio interface for direct topic publishing

        # Audio parameters
        self.SAMPLE_RATE = 16000
        self.CHANNELS = 1
        self.BIT_RESOLUTION = 16
        self.BYTES_PER_SAMPLE = self.BIT_RESOLUTION // 8
        
        # Dialog session management
        self.dialog_session: Optional[ROSDialogSession] = None
        self.is_running = False
        self.event_loop = None
        self.session_thread = None
        self.ws_config = None
        self.thread_exception = None  # 保存线程异常
        self.exception_queue = Queue()  # 异常消息队列
        
        # Audio queue for ROS microphone data
        self.audio_queue = queue.Queue()
        
        # 错误监控线程
        self.monitor_thread = None
        self.should_monitor = False

        self._last_app_id = None
        self._last_access_key = None
        
        # 重启相关
        self._restart_count = 0
        self.MAX_RESTARTS = 3
        self.first_connection = True

        print("[Speech] RobotLLMDoubaoCore initialized")

    def _setup_websocket_config(self, app_id: str, access_key: str):
        """Setup WebSocket configuration with provided credentials"""
        self.ws_config = {
            "base_url": "wss://openspeech.bytedance.com/api/v3/realtime/dialogue",
            "headers": {
                "X-Api-App-ID": app_id,
                "X-Api-Access-Key": access_key,
                "X-Api-Resource-Id": "volc.speech.dialog",
                "X-Api-App-Key": "PlgvMymc7f3tQnJ6",
                "X-Api-Connect-Id": config.ws_connect_config["headers"]["X-Api-Connect-Id"],
            }
        }

    def _run_async_session(self):
        """Run dialog session in separate thread with its own event loop"""
        self.event_loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.event_loop)
        
        try:
            self.event_loop.run_until_complete(self._async_session_main())
        except Exception as e:
            print(f"[Speech] Dialog session error: {e}")
        finally:
            self.event_loop.close()

    async def _async_session_main(self):
        """Main async session handler"""       
        try:
            # Establish WebSocket connection first (reconnect after test)
            connection_success = await self.dialog_session.client.start_connection()
            if not connection_success:
                print("[Speech] Failed to establish WebSocket connection in session")
                return

            await self.dialog_session.client.start_session()
            print("[Speech] Speech session started successfully")
            
            # Start receiving responses
            receive_task = asyncio.create_task(self.dialog_session.receive_loop())
            
            # Start processing ROS microphone data
            audio_task = asyncio.create_task(self._process_ros_microphone_data())
            
            # Send hello message
            if self.first_connection:
                await self.dialog_session.client.say_hello()
            
            # Wait for session to finish
            while self.is_running and not self.dialog_session.is_session_finished:
                await asyncio.sleep(0.1)
                
            # Clean up tasks
            receive_task.cancel()
            audio_task.cancel()
            
            # Finish session
            await self.dialog_session.client.finish_session()
            while not self.dialog_session.is_session_finished:
                await asyncio.sleep(0.1)
            await self.dialog_session.client.finish_connection()
            await self.dialog_session.client.close()
            
            print(f"[Speech] Dialog session ended, logid: {self.dialog_session.client.logid}")
            
        except Exception as e:
            print(f"[Speech] Session error: {e}")
        finally:
            if self.dialog_session:
                self.dialog_session.cleanup()

    async def _process_ros_microphone_data(self):
        """Process microphone data from ROS topic"""
        print("[Speech] Starting ROS microphone data processing")
        
        while self.is_running:
            try:
                # Get audio data from ROS microphone
                audio_data = self.microphone.get_data()
                
                if audio_data is not None and len(audio_data) > 0:
                    # Convert numpy array to bytes if needed
                    if isinstance(audio_data, np.ndarray):
                        audio_bytes = audio_data.tobytes()
                    else:
                        audio_bytes = audio_data
                    
                    # Send audio data to dialog service
                    await self.dialog_session.client.task_request(audio_bytes)
                    
                await asyncio.sleep(0.01)  # Small delay to prevent CPU overload
                
            except Exception as e:
                print(f"[Speech] Error processing ROS microphone data: {e}")
                await asyncio.sleep(0.1)

    def verify_connection(self, app_id: str, access_key: str) -> bool:
        """Set the app ID and access key for the speech system."""
        if not app_id or not access_key:
            print("[Speech] App ID and Access Key are required")
            return False

        self._last_app_id = app_id
        self._last_access_key = access_key

        # Setup WebSocket configuration
        self._setup_websocket_config(app_id, access_key)
        # Use custom ROS-integrated DialogSession with Audio interface
        self.dialog_session = ROSDialogSession(self.ws_config, enable_signal_handler=False)

        # Test connection using event loop
        try:
            import asyncio
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            
            try:
                connection_successful = loop.run_until_complete(self.dialog_session.client.start_connection())
                if connection_successful:
                    # Close the test connection since we'll reconnect in _async_session_main
                    loop.run_until_complete(self.dialog_session.client.close())
                    print("[Speech] WebSocket connected successfully")
                    return True
                else:
                    print("[Speech] WebSocket connection failed")
                    self.dialog_session = None  # Clear failed session
                    return False
            finally:
                loop.close()
                
        except Exception as e:
            print(f"[Speech] Failed to test WebSocket connection: {e}")
            self.dialog_session = None  # Clear failed session
            return False

    def start_speech_system(self):
        """Start the speech dialog system with Doubao service."""
        # 检查是否正在运行（更智能的检查）
        if self.is_running:
            # 检查线程是否真的在运行
            if self.session_thread and self.session_thread.is_alive():
                print("[Speech] Speech system is already running")
                return True
            else:
                print("[Speech] 状态不一致，重置为未运行")
                self.is_running = False
        
        if self.dialog_session is None:
            print("[Speech] Dialog session not initialized.")
            # 尝试自动重新连接
            if self._last_app_id and self._last_access_key:
                print("[Speech] 尝试自动重新连接...")
                success = self.verify_connection(self._last_app_id, self._last_access_key)
                if not success:
                    print("[Speech] 自动重新连接失败")
                    return False
            else:
                print("[Speech] 请先调用verify_connection()")
                return False
        
        try:
            print(f"[Speech] Starting speech system")
            
            # 重置状态
            self.is_running = True
            self.thread_exception = None
            while not self.exception_queue.empty():
                self.exception_queue.get()
            
            # 创建并启动语音线程
            self.session_thread = threading.Thread(
                target=self._safe_run_async_session,
                name="DoubaoSpeechThread"
            )
            self.session_thread.daemon = True
            self.session_thread.start()
            
            # 启动错误监控
            if not self.should_monitor:
                self._start_error_monitor()
            
            # 等待连接建立
            time.sleep(2)
            
            # 检查线程是否启动成功
            if not self.session_thread.is_alive():
                print("[Speech] 警告：语音线程启动失败")
                self.is_running = False
                return False
                
            print("[Speech] Speech system started successfully")
            return True
            
        except Exception as e:
            print(f"[Speech] Failed to start speech system: {e}")
            self.is_running = False
            return False

    def stop_speech_system(self):
        """Stop the Doubao speech system."""
        print("[Speech] Stopping speech system")
        
        # 1. 先停止标志
        self.is_running = False
        self.should_monitor = False
        
        # 2. 停止dialog session
        if self.dialog_session:
            try:
                self.dialog_session.is_running = False
                self.dialog_session.is_recording = False
                self.dialog_session.is_playing = False
            except:
                pass
                
        # 3. 等待语音线程结束（先处理这个）
        if self.session_thread and self.session_thread.is_alive():
            print("[Speech] 等待语音线程结束...")
            try:
                self.session_thread.join(timeout=5)
                if self.session_thread.is_alive():
                    print("[Speech] 警告：语音线程仍在运行")
                else:
                    print("[Speech] 语音线程已结束")
            except Exception as e:
                print(f"[Speech] 等待语音线程时出错: {e}")
        
        # 4. 等待监控线程结束（放在最后，确保不是当前线程）
        if (self.monitor_thread and 
            self.monitor_thread.is_alive() and 
            self.monitor_thread != threading.current_thread()):  # 关键检查！
            print("[Speech] 等待监控线程结束...")
            try:
                self.monitor_thread.join(timeout=2)
                print("[Speech] 监控线程已结束")
            except Exception as e:
                print(f"[Speech] 等待监控线程时出错: {e}")
        
        print("[Speech] Speech system stopped successfully")

    def is_system_running(self) -> bool:
        """Check if the speech system is currently running."""
        return self.is_running

    def get_session_status(self) -> dict:
        """Get current session status information."""
        status = {
            "is_running": self.is_running,
            "has_session": self.dialog_session is not None,
            "session_finished": False,
            "logid": ""
        }
        
        if self.dialog_session:
            status["session_finished"] = self.dialog_session.is_session_finished
            if self.dialog_session.client:
                status["logid"] = self.dialog_session.client.logid
                
        return status
    
    def get_status(self):
        """获取系统状态"""
        status = {
            "is_running": self.is_running,
            "thread_alive": self.session_thread.is_alive() if self.session_thread else False,
            "monitor_alive": self.monitor_thread.is_alive() if self.monitor_thread else False,
            "has_error": self.thread_exception is not None,
            "error_message": str(self.thread_exception) if self.thread_exception else None,
            "pending_errors": self.exception_queue.qsize(),
            "restart_count": self._restart_count,
            "max_restarts": self.MAX_RESTARTS,
            "has_credentials": bool(self._last_app_id and self._last_access_key),
            "has_session": self.dialog_session is not None
        }
        return status
    
    def _safe_run_async_session(self):
        """安全的线程执行函数，捕获所有异常"""
        try:
            print("[Speech] 语音线程开始执行")
            self._run_async_session()
        except Exception as e:
            print(f"[Speech] 语音线程发生严重错误: {e}")
            traceback.print_exc()
            
            # 将异常信息放入队列，让监控线程知道
            self.exception_queue.put({
                'type': 'thread_error',
                'exception': e,
                'traceback': traceback.format_exc()
            })
            
            # 设置异常标志
            self.thread_exception = e
            
            # 尝试清理
            try:
                if self.dialog_session:
                    self.dialog_session.cleanup()
            except:
                pass
        finally:
            print("[Speech] 语音线程结束")

    def _handle_thread_error(self):
        """处理线程错误，自动关闭系统"""
        print("[Speech] 开始处理线程错误...")
        
        # 1. 停止监控
        self.should_monitor = False
        
        # 2. 调用stop_speech_system
        try:
            self.stop_speech_system()
            print("[Speech] 已自动关闭语音系统")
        except Exception as e:
            print(f"[Speech] 关闭系统时出错: {e}")
            self._notify_error_occurred()
            return  # 关闭失败就不重启
        
        # 3. 等待一段时间让系统完全停止
        time.sleep(1)
        
        # 4. 检查是否可以重启（限制次数）
        if self._restart_count < self.MAX_RESTARTS:
            self._restart_count += 1
            print(f"[Speech] 尝试自动重启 ({self._restart_count}/{self.MAX_RESTARTS})...")
            
            # 调用安全重启方法
            restart_success = self._safe_restart()
            if restart_success:
                print("[Speech] 自动重启成功")
                return
            else:
                print("[Speech] 自动重启失败")
        else:
            print(f"[Speech] 已达到最大重启次数({self.MAX_RESTARTS})，停止重启")
            
        # 5. 通知外部
        self._notify_error_occurred()

    def _notify_error_occurred(self):
        """通知外部发生了错误（可以扩展为事件或回调）"""
        print("[Speech] 系统因错误而关闭")

    def _safe_restart(self):
        """安全重启语音系统"""
        try:
            print("[Speech] 执行安全重启流程...")
            
            # 1. 清理旧的会话
            if self.dialog_session:
                try:
                    self.dialog_session.cleanup()
                    print("[Speech] 清理旧会话完成")
                except Exception as e:
                    print(f"[Speech] 清理会话时出错: {e}")
                finally:
                    self.dialog_session = None
            
            # 2. 重置状态
            self.is_running = False
            self.thread_exception = None
            self.session_thread = None
            while not self.exception_queue.empty():
                self.exception_queue.get()
            
            # 3. 检查是否有保存的凭证
            if not self._last_app_id or not self._last_access_key:
                print("[Speech] 错误：没有保存的连接凭证，无法重启")
                return False
            
            # 4. 重新连接
            print(f"[Speech] 使用保存的凭证重新连接...")
            success = self.verify_connection(self._last_app_id, self._last_access_key)
            if not success:
                print("[Speech] 重新连接失败")
                return False
            
            # 5. 等待一下
            #time.sleep(1)
            
            # 6. 启动系统
            print("[Speech] 启动语音系统...")
            self.first_connection = False
            success = self.start_speech_system()
            
            if success:
                print("[Speech] 重启成功")
                self._restart_count = 0  # 成功则重置计数
                return True
            else:
                print("[Speech] 启动失败")
                return False
                
        except Exception as e:
            print(f"[Speech] 安全重启过程中出错: {e}")
            traceback.print_exc()
            return False

    def _start_error_monitor(self):
        """启动错误监控线程 - 修复版"""
        def monitor():
            print("[Speech] 启动错误监控线程")
            
            while self.should_monitor:
                try:
                    # 1. 检查异常队列（非阻塞）
                    has_queue_error = False
                    try:
                        error_info = self.exception_queue.get_nowait()
                        if error_info['type'] == 'thread_error':
                            has_queue_error = True
                            print(f"[Speech] 检测到线程异常: {error_info['exception']}")
                    except:
                        pass  # 队列为空
                    
                    # 2. 检查线程是否存活
                    thread_dead = (self.session_thread and 
                                not self.session_thread.is_alive() and 
                                self.is_running)
                    
                    # 3. 如果发现错误
                    if (has_queue_error or thread_dead) and self.should_monitor:
                        print(f"[Speech] 检测到系统异常，准备处理...")
                        
                        # 短暂延迟，避免误判
                        time.sleep(0.5)
                        
                        # 再次确认
                        if thread_dead or has_queue_error:
                            print("[Speech] 确认异常，执行错误处理")
                            # 在独立线程中执行关闭，避免阻塞当前监控线程
                            def safe_handle_error():
                                try:
                                    self._handle_thread_error()
                                except Exception as e:
                                    print(f"[Speech] 错误处理时出错: {e}")
                            
                            error_handler_thread = threading.Thread(
                                target=safe_handle_error,
                                name="ErrorHandlerThread",
                                daemon=True
                            )
                            error_handler_thread.start()
                            
                            # 当前监控线程可以退出了
                            break
                    
                    # 休眠一段时间再检查
                    time.sleep(2)
                    
                except Exception as e:
                    print(f"[Speech] 监控线程异常: {e}")
                    time.sleep(5)
            
            print("[Speech] 错误监控线程结束")
        
        # 启动监控线程
        self.should_monitor = True
        self.monitor_thread = threading.Thread(
            target=monitor,
            name="ErrorMonitorThread",
            daemon=True
        )
        self.monitor_thread.start()


