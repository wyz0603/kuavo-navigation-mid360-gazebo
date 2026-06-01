#!/usr/bin/env python

from __future__ import print_function
from edgeboard_service.srv import EbMessage,EbMessageResponse #注意是功能包名.srv
import rospy

def handle_edgeboard_yolo(req):
    my_result = "OK"
    print("Accept config:",req.config)
    print("Return result:",my_result)
    return EbMessageResponse(my_result)

def edgeboard_yolo_server():
    rospy.init_node('edgeboard_yolo_server')
    s = rospy.Service('edgeboard_yolo', EbMessage, handle_edgeboard_yolo)
    print("Ready to edgeboard yolo.")
    rospy.spin()

if __name__ == "__main__":
    edgeboard_yolo_server()

