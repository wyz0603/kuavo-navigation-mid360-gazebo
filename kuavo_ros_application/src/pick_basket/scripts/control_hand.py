#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import time
from enum import Enum
from kuavoRobotSDK import kuavo

class ControlHand(object):
    class Hand(Enum):
        """定义手部选择枚举"""
        Right = 0
        Left = 1
        Both = 2

    def __init__(self):
        """初始化ControlHand类实例"""
        self._kuavo = kuavo("control_hand")
        self._zero_pose = [0] * 6
        self._poses = {
            'pick': [100, 100, 95, 95, 95, 95], # 抓取
            'open': [0, 100, 0, 0, 0, 0],       # 打开
            'fist': [65, 65, 90, 80, 80, 90],   # 握拳
            'ok': [65, 65, 60, 0, 0, 0]         # OK
        }

    def _set_hand_position(self, pose_name, hand: Hand = Hand.Right):
        rospy.loginfo(f"set_hand_position: {pose_name}")
        pose = self._poses[pose_name]
        if hand == self.Hand.Right:
            self._kuavo.set_end_hand(self._zero_pose, pose)
        elif hand == self.Hand.Left:
            self._kuavo.set_end_hand(pose, self._zero_pose)
        else:
            self._kuavo.set_end_hand(pose, pose)

    def release(self):
        rospy.loginfo("set_hand_position: release")
        self._kuavo.set_end_hand(self._zero_pose, self._zero_pose)

    def pick(self, hand: Hand = Hand.Right):
        self._set_hand_position('pick', hand)

    def open(self, hand: Hand = Hand.Right):
        self._set_hand_position('open', hand)

    def fist(self, hand: Hand = Hand.Right):
        self._set_hand_position('fist', hand)

    def ok(self, hand: Hand = Hand.Right):
        self._set_hand_position('ok', hand)

    def control(self, hand, action):
        self._set_hand_position(action, hand)  
        
    def control_end_hand(self, l_hand_pos, r_hand_pos):
        """
        直接发送关节值控制末端灵巧手
        """
        self._kuavo.set_end_hand(l_hand_pos, r_hand_pos)
    
if __name__ == '__main__':
    # Debug
    rospy.init_node('control_hand_node')
    control_hand = ControlHand()
    control_hand.pick()
    print("control hand: Pick")
    time.sleep(1)
    print("control hand: Release")
    control_hand.release()
    time.sleep(1)
    print("control hand: Fist")
    control_hand.fist()
    time.sleep(1)
    control_hand.release()
