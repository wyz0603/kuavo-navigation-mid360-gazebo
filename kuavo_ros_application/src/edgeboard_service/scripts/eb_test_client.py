#!/usr/bin/env python

from __future__ import print_function 

import sys
import rospy
from edgeboard_service.srv import * #注意是功能包名.srv

def edgeboard_yolo_client(my_config):
    rospy.wait_for_service('edgeboard_yolo')
    try:
        edgeboard_yolo = rospy.ServiceProxy('edgeboard_yolo', EbMessage)
        resp1 = edgeboard_yolo(my_config)
        return resp1.result
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

if __name__ == "__main__":

    my_config = "my_config"
    print("Send config:%s",my_config)
    print("Accept result:",edgeboard_yolo_client(my_config))

