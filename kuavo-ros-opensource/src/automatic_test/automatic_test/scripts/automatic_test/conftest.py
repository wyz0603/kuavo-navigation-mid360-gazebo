from enum import Enum
from datetime import datetime
import pytest
from types import SimpleNamespace
import time
import rospy
import subprocess
import threading
from typing import Optional, List
import os
from trace_path.mpc_path_tracer import MpcPathTracer
from multiprocessing import Process
import requests
from .constant import test_project_cn_name_dict
from std_msgs.msg import Int8

def pytest_configure(config):
    config.addinivalue_line("markers", "walk: mark test as a walk test")
    config.addinivalue_line("markers", "single_step_control: mark test as a single step control test")


@pytest.fixture(scope="class")
def ros_setup():
    """Initialize a ROS node for testing.
    Using class scope to ensure the node is only initialized once for each test class.
    """
    rospy.init_node('test_node', anonymous=True)
    yield



def mpc_tracer():
    """Create MPC Path Tracer instance.
    Using session scope to ensure only one instance exists.
    Depends on ros_node fixture to ensure ROS is initialized.
    """
    rospy.init_node('mpc_path_tracer_node')
    tracer = MpcPathTracer('test_mpc_tracer_node')
    rospy.spin()

@pytest.fixture(scope="session")
def mpc_tracer_process():
    """Create MPC Path Tracer instance as a separate process.
    Using session scope to ensure only one instance exists.
    Depends on ros_node fixture to ensure ROS is initialized.
    """
    process = Process(target=mpc_tracer)
    process.start()
    yield process
    process.terminate()


def get_node_list() -> List[str]:
    """Get list of running ROS nodes"""
    try:
        cmd = "rosnode list"
        result = subprocess.check_output(cmd, shell=True, text=True)
        nodes = [node.strip() for node in result.split('\n') if node.strip()]
        return nodes
    except subprocess.CalledProcessError:
        return []

def get_node_pid(node_name: str, max_retries=3, retry_delay=1.0) -> Optional[int]:
    """Get PID of a ROS node using rosnode info command with retry mechanism"""
    for attempt in range(max_retries):
        try:
            if not node_name.startswith('/'):
                node_name = '/' + node_name
            
            cmd = f"rosnode info {node_name} | grep Pid"
            result = subprocess.check_output(cmd, shell=True, text=True)
            pid = int(result.strip().split(": ")[1])
            return pid
        except (subprocess.CalledProcessError, IndexError, ValueError) as e:
            if attempt < max_retries - 1:
                rospy.logwarn(f"Failed to get PID for {node_name}, attempt {attempt + 1}/{max_retries}")
                rospy.sleep(retry_delay)
            else:
                rospy.logerr(f"Failed to get PID for {node_name} after {max_retries} attempts: {e}")
                return None

class RobotMonitor:
    def __init__(self):
        self.is_running = True
        self.robot_alive = True
        self.monitor_thread = None
        self.lock = threading.Lock()
        self.last_error = None

    def check_status(self):
        node_name = "/nodelet_manager"
        while self.is_running:
            try:
                # Step 1: Check if node exists
                nodes = get_node_list()
                node_found = False
                for node in nodes:
                    if node.endswith(node_name):
                        node_found = True
                        node_name_full = node
                        break

                if not node_found:
                    with self.lock:
                        self.robot_alive = False
                        self.last_error = f"机器人主节点死亡💀"
                    break

                # Step 2: Get node PID
                pid = get_node_pid(node_name_full)
                if pid is None:
                    with self.lock:
                        self.robot_alive = False
                        self.last_error = f"机器人主节点PID获取失败💀"
                    break

                time.sleep(1)

            except Exception as e:
                with self.lock:
                    self.robot_alive = False
                    self.last_error = f"机器人节点监控异常: {e}"
                break

    def start(self):
        self.monitor_thread = threading.Thread(target=self.check_status)
        self.monitor_thread.daemon = True
        self.monitor_thread.start()

    def stop(self):
        self.is_running = False
        if self.monitor_thread and self.monitor_thread.is_alive():
            self.monitor_thread.join(timeout=2)

    def get_status(self):
        with self.lock:
            return self.robot_alive

    def get_error(self):
        with self.lock:
            return self.last_error

@pytest.fixture(scope="function")
def check_robot_alive():
    """Check if the robot is alive using a monitoring thread.
    The thread continuously checks:
    1. nodelet_manager node exists
    2. node has a valid PID
    3. process with that PID exists
    """
    monitor = RobotMonitor()
    monitor.start()
    
    try:
        time.sleep(0.5)
        
        yield monitor
        
    finally:
        monitor.stop()

@pytest.fixture(scope="function")
def check_robot_ready():
    """Check if the robot is ready 
    The thread continuously checks:
    1. robot_ready parameter is true
    """
    try:
        while not rospy.is_shutdown() and not rospy.get_param('/robot_ready', False):
            rospy.loginfo("Waiting for robot to be ready")
            time.sleep(1)
            
        if rospy.is_shutdown():
            pytest.exit("ROS has been shutdown")
            
    except (rospy.ROSInterruptException, KeyboardInterrupt):
        pytest.exit("Test interrupted by user")
    
    yield


def periodic_notify(test_name, start_time, stop_event):
    """Periodically notify test duration every 5 minutes"""
    while not stop_event.is_set():
        # Sleep for 5 minutes
        if stop_event.wait(300):  # 300 seconds = 5 minutes
            break
            
        current_time = datetime.now()
        duration = (current_time - start_time).total_seconds()
        
        notify_str = f"""
        🕒 测试运行状态
        📌 测试项目: {test_project_cn_name_dict[test_name]}
        ⏱️ 已运行时长: {duration/60:.1f} 分钟
        """
        notify_wecom_bot(notify_str)

@pytest.fixture(scope="function")
def test_timer(request):
    """Record test start time, end time, duration and result."""
    test_name = request.node.name
    test_module = request.module.__name__
    test_class = request.cls.__name__ if request.cls else None
    test_marks = [mark.name for mark in request.node.iter_markers()]
    
    start_time = datetime.now()
    
    stop_notify = threading.Event()
    notify_thread = threading.Thread(
        target=periodic_notify,
        args=(test_name, start_time, stop_notify)
    )
    notify_thread.daemon = True
    notify_thread.start()
    
    notify_str = f"""
    🚀 测试项目: {test_project_cn_name_dict[test_name]}
    🕒 开始时间: {start_time.strftime("%Y-%m-%d %H:%M:%S")}
    """
    notify_wecom_bot(notify_str)
    
    yield SimpleNamespace(
        start_time=start_time,
        test_name=test_name,
        test_module=test_module,
        test_class=test_class,
        marks=test_marks
    )
    
    stop_notify.set()
    notify_thread.join(timeout=1)
    
    end_time = datetime.now()
    duration = (end_time - start_time).total_seconds()
    
    report = request.node.rep_call
    result = "PASSED" if not report.failed else "FAILED"
    
    
    notify_str = f"""
    🎯 测试项目: {test_project_cn_name_dict[test_name]}
    ✅ 结果: {result} {'🎉' if result == 'PASSED' else '🔥'}
    ⏱️ 持续时间: {duration/60:.1f} 分钟
    """
    
    if report.failed:
        if report.longrepr:
            error_full = str(report.longrepr)
            error_msg = None
            
            # Extract error message based on different error patterns
            if "AssertionError:" in error_full:
                error_msg = error_full.split("AssertionError: ")[-1].split("\n")[0]
            elif "Exception:" in error_full:
                error_msg = error_full.split("Exception: ")[-1].split("\n")[0]
            elif "Error:" in error_full:
                error_msg = error_full.split("Error: ")[-1].split("\n")[0]
            
            # Categorize and format error message
            if error_msg:
                notify_str += f"❌ 失败信息: {error_msg}"

    notify_wecom_bot(notify_str)

@pytest.hookimpl(tryfirst=True, hookwrapper=True)
def pytest_runtest_makereport(item, call):
    """Hook to capture test results"""
    outcome = yield
    rep = outcome.get_result()
    setattr(item, "rep_" + rep.when, rep)

def notify_wecom_bot(str):
    webhook_url = os.environ.get("AUTOMATIC_TEST_WEBHOOK_URL")
    robot_serial_number = os.environ.get("ROBOT_SERIAL_NUMBER")
    if not webhook_url:
        rospy.logerr("AUTOMATIC_TEST_WEBHOOK_URL is not set")
        return
    str = f"机器人编号: {robot_serial_number}\n{str}"
    data = {
        "msgtype": "text",
        "text": {
            "content": str
        },
    }
    requests.post(webhook_url, json=data)

class AutoTrackingStats(Enum):
    NOT_FOLLOWING = 0
    FOLLOWING = 1

global auto_tracking_stats

class AutoTrackingMonitor:
    def __init__(self):
        self._status = AutoTrackingStats.FOLLOWING
        self.lock = threading.Lock()
        
    def callback(self, msg):
        with self.lock:
            if msg.data == AutoTrackingStats.FOLLOWING.value:
                self._status = AutoTrackingStats.FOLLOWING
            else:
                self._status = AutoTrackingStats.NOT_FOLLOWING
            print(f"Status updated: {self._status}")
    
    @property
    def status(self):
        with self.lock:
            return self._status

@pytest.fixture(scope="function")
def auto_tracking_stats():
    monitor = AutoTrackingMonitor()
    auto_tracking_stats_sub = rospy.Subscriber('/auto_tracking_stats', Int8, monitor.callback)
    yield monitor
    auto_tracking_stats_sub.unregister()