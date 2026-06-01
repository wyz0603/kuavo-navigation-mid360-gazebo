from src.nimservosSDK import NiMServos
import time
import multiprocessing
import threading
import sys
import os

current_path = os.path.dirname(os.path.abspath(__file__))
nimservossdk_path = os.path.join(current_path, "LD_NIMSERVOS_SDK_PATH")
sys.path.append(nimservossdk_path) 

# current_ld_library_path = os.environ.get("LD_LIBRARY_PATH", "")
# os.environ["LD_LIBRARY_PATH"] = f"{library_path}:{current_ld_library_path}"

def Test_nimservos_PP(id=1):
    # 初始化电机参数
    motor.initializeNiMServosMotor(id)
    # 初始化点击PP模式
    motor.nimservos_PP_Init(
        device_id=id,
        postion=342082256,
        speed=50000,
        acceleration=40000,
        deceleration=40000
    )
    pos = 332082256
    while True:
        motor.nimservos_PP_UpdatePos(1,pos)
        time.sleep(1)
        pos = pos - 10000
        

def Test_nimservos_VM(id=1):
    # 初始化电机
    motor.initializeNiMServosMotor(id)
    # 初始化VM模式
    motor.nimservos_VM_Init(
        device_id=id,
        speed=0,
    )
    speed = 0
    while True:
        motor.nimservos_VM_UpdateSpeed(1,speed)
        time.sleep(1)
        speed = speed + 100


def Test_nimservos_PT(id=1):
    motor.initializeNiMServosMotor(id)

    motor.nimservos_PT_Init(
        device_id=id,
        torque=2000,
        torque_ramp=10,
        torque_ramp_type=2
    )
    torque = 0
    while True:
        motor.nimservos_PT_Update(1,torque)
        time.sleep(0.1)
        torque = torque + 10

def Test_nimservos_CSP(id=1):
    motor.initializeNiMServosMotor(id)
    pos = 337489783
    motor.nimservos_CSP_Init(1,pos)
    while True:
        motor.nimservos_CSP_UpdatePos(1,pos)
        time.sleep(1)
        pos = pos +100000


def Test_nimservos_CSV(id=1):
    motor.initializeNiMServosMotor(id) 
    s = 100000
    motor.nimservos_CSV_Init(device_id=1,speed=s)
    while True:
        motor.nimservos_CSV_UpdateSpeed(1,s)
        s = s + 1000
        time.sleep(0.5)
        # motor.sendCANopenMessage(0x601,0x23,0x60FF,0x00,speed)

def Test_nimservos_CST(id=1):
    motor.initializeNiMServosMotor(id) 
    t = 0   
    motor.nimservos_CST_Init(device_id=1,torque = t)
    while True:
        motor.nimservos_CST_UpdateTorque(1,t)
        time.sleep(1)
        t = t + 10

def Test_SetSpeed():
    # speed = 100
    pos = 1000000
    while True:
        # speed = speed + 10
        # motor.sendCANopenMessage(0x601,0x2B,0x6042,0x00,speed)
        motor.CiA402ParameterTypeDef["target_position"] = pos 
        motor.prepareMotor(0x601)
        motor.disableMotor(0x601)
        motor.enableMotor(0x601,"PP")
        # 触发电机运行
        motor.triggerMotorOperation(0x601,"abs_pos_delay_update")   
        time.sleep(1)  
        
def Test_RecvProcess():
    while True:
        # data = motor.receiveCANopenMessage()
        data = motor.nimservos_readSpeed(1)
        # data = motor.nimservos_readAbsolutePos(1)
        print(f"[DEBUG]:{data}")
        print("------------------------------------------:")
        time.sleep(0.1)  

if __name__ == "__main__":

    threadings = []
    processes = []
    # create a NiMServos object 
    print("nimservos test")
    motor = NiMServos()
    motor.cantool.close_canbus()
    # open can
    can_open_status = motor.cantool.open_canbus()
    print(can_open_status)

    if can_open_status:
        print("[MESSAGW] : can open successes")
        
        # Test_nimservos_PT()
        # Test_nimservos_PP()
        # Test_nimservos_VM()

        # 创建can监听进程
        # control_thread = threading.Thread(target=Test_nimservos_VM)
        recv_control_thread = threading.Thread(target=Test_RecvProcess)
        setspeed_thread = threading.Thread(target=Test_nimservos_CSV)
        # threadings.append(recv_control_thread)
        # threadings.append(control_thread)
        # threadings.append(setspeed_thread)
        # control_thread.start()
        recv_control_thread.start()
        setspeed_thread.start()

        recv_control_thread.join()



