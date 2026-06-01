#!/usr/bin/env python3
import rospy
from kuavo_msgs.srv import handForceLevel, handForceLevelRequest

def change_force_level(hand_side, force_level):
    """
    Change force level for specified hand(s)
    Args:
        hand_side: LEFT_HAND(0), RIGHT_HAND(1), or BOTH_HANDS(2)
        force_level: 0-3, where 0 is minimum force and 3 is maximum force
    Returns:
        success: bool indicating if service call succeeded
        message: status message from service
    """
    rospy.wait_for_service('dexhand/change_force_level')
    try:
        change_force = rospy.ServiceProxy('dexhand/change_force_level', handForceLevel)
        req = handForceLevelRequest()
        req.hand_side = hand_side
        req.force_level = force_level
        response = change_force(req)
        return response.success, response.message
    except rospy.ServiceException as e:
        print(f"Service call failed: {e}")
        return False, str(e)

if __name__ == '__main__':
    rospy.init_node('change_force_level_client')
    
    # Example usage:
    success, msg = change_force_level(handForceLevelRequest.BOTH_HANDS, handForceLevelRequest.FULL)
    print(f"Success: {success}, Message: {msg}")
