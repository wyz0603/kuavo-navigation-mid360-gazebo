#!/usr/bin/env python3
import time
import rospy
from kuavo_msgs.srv import enableHandTouchSensor, enableHandTouchSensorRequest

def enable_touch_sensor(hand:str, mask):
    service_name = f'dexhand/{hand}/enable_touch_sensor'
    try:
        rospy.wait_for_service(service_name, timeout=2.0)
        enable_touch = rospy.ServiceProxy(service_name, enableHandTouchSensor)
        req = enableHandTouchSensorRequest()
        req.mask = mask
        response = enable_touch(req)
        return response.success, response.message
    except rospy.ServiceException as e:
        print(f"Service call failed: {e}")
        return False, str(e)
    

if __name__ == '__main__':
    rospy.init_node('enable_touch_sensor_client')

    # enable right hand thumb and index finger touch sensor
    success, msg = enable_touch_sensor('right', 0b00000011)
    print(f"Success: {success}, Message: {msg}")
    
    # enable left hand thumb and index finger touch sensor
    success, msg = enable_touch_sensor('left', 0b00000011)
    print(f"Success: {success}, Message: {msg}")

    # disable all touch sensor
    # success, msg = enable_touch_sensor('right', 0b00000000)
    # print(f"Success: {success}, Message: {msg}")
