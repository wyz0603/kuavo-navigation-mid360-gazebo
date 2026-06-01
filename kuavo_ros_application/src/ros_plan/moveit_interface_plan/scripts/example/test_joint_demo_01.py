#!/usr/bin/env python3
# -*- coding: UTF-8 -*-


import sys
import moveit_commander
import rospy

from planner import Planner
from logger import Logger
from publisher import Publisher

from utils import angle_to_rad

if __name__ == "__main__":
    rospy.init_node("test_node", anonymous=True)
    moveit_commander.roscpp_initialize(sys.argv)

    planner = Planner()
    logger = Logger()
    publisher = Publisher()
    
    # publisher.start_auto_publish()
    logger.make_traj_dir()

    print("=====================================================")
    planner.set_start_state([0, 0, 0, 0, 0, 0, 0])
    traj = planner.plan_to_target_joints(angle_to_rad([119, 177, 175, 16, 0, 0, 0]))
    logger.dump_traj(traj, file_name="test1")
    
    # print("=====================================================")
    # planner.set_start_state([0.2, 0, 0.5, 0.1, 0, 0, 0])
    # traj = planner.plan_to_target_joints([0.3, 0.6, 0.8, 0.8, 0, 0, 0])
    # logger.dump_traj(traj, file_name="test2")
    
    #publisher.stop_auto_publish()
    
