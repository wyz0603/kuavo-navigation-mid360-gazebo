#!/usr/bin/env python3
# -*- coding: UTF-8 -*-


import sys
import moveit_commander
import rospy

from planner import Planner
from logger import Logger
from publisher import Publisher

from utils import angle_to_rad

Point_zero = angle_to_rad([0, 0, 0, 0, 0, 0, 0])

Point_1 = angle_to_rad([ 20, 50, 0,   0, 10,   0, 0])
Point_2 = angle_to_rad([ 30, 90, 0, -50, 90, -30, 0])
Point_3 = angle_to_rad([  0, 90, 0, -50, 90, -30, 0])
Point_4 = angle_to_rad([-35, 10, 0, -30,  0, -30, 0])
Point_5 = angle_to_rad([-35,  0, 0, -20,  0, -50, 0])
Point_6 = angle_to_rad([-50,  0, 0, -40,  0, -20, 0])

if __name__ == "__main__":
    rospy.init_node("test_node", anonymous=True)
    moveit_commander.roscpp_initialize(sys.argv)

    planner = Planner()
    logger = Logger()
    publisher = Publisher()
    
    publisher.start_auto_publish()
    logger.make_traj_dir()

    print("=====================================================")
    planner.set_start_state(Point_zero)
    traj = planner.plan_to_target_joints(Point_1)
    logger.dump_traj(traj, file_name="test1")

    print("=====================================================")
    planner.set_start_state(Point_1)
    traj = planner.plan_to_target_joints(Point_2)
    logger.dump_traj(traj, file_name="test2")


    print("=====================================================")
    planner.set_start_state(Point_2)
    traj = planner.plan_to_target_joints(Point_3)
    logger.dump_traj(traj, file_name="test3")


    print("=====================================================")
    planner.set_start_state(Point_3)
    traj = planner.plan_to_target_joints(Point_4)
    logger.dump_traj(traj, file_name="test4")
    
    print("=====================================================")
    planner.set_start_state(Point_4)
    traj = planner.plan_to_target_joints(Point_5)
    logger.dump_traj(traj, file_name="test5")

    print("=====================================================")
    planner.set_start_state(Point_5)
    traj = planner.plan_to_target_joints(Point_6)
    logger.dump_traj(traj, file_name="test6")

    print("=====================================================")
    print("=====================================================")
    print("=====================================================")


    print("=====================================================")
    planner.set_start_state(Point_6)
    traj = planner.plan_to_target_joints(Point_5)
    logger.dump_traj(traj, file_name="test7")

    print("=====================================================")
    planner.set_start_state(Point_5)
    traj = planner.plan_to_target_joints(Point_4)
    logger.dump_traj(traj, file_name="test8")

    print("=====================================================")
    planner.set_start_state(Point_4)
    traj = planner.plan_to_target_joints(Point_3)
    logger.dump_traj(traj, file_name="test9")

    print("=====================================================")
    planner.set_start_state(Point_3)
    traj = planner.plan_to_target_joints(Point_2)
    logger.dump_traj(traj, file_name="test10")

    print("=====================================================")
    planner.set_start_state(Point_2)
    traj = planner.plan_to_target_joints(Point_1)
    logger.dump_traj(traj, file_name="test11")

    print("=====================================================")
    planner.set_start_state(Point_1)
    traj = planner.plan_to_target_joints(Point_zero)
    logger.dump_traj(traj, file_name="test12")

    publisher.stop_auto_publish()