#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import threading
import math
import time
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from pick_basket.msg import planArmState
from pick_basket.srv import planArmTrajectoryCubicSpline, planArmTrajectoryCubicSplineRequest
from ocs2_msgs.msg import mpc_observation

class CubicsplinePlanner:
    def __init__(self):
        """ cubic spline """
        self._srv_plan_arm_traj_cubicspline = rospy.ServiceProxy('/cubic_spline/plan_arm_trajectory', planArmTrajectoryCubicSpline)
        self._sub_cubicspline_arm_traj_state = rospy.Subscriber("/cubic_spline/arm_traj_state", planArmState, self.cubicspline_traj_callback)
        self.cubicspline_traj_sub = rospy.Subscriber('/cubic_spline/arm_traj', JointTrajectory, self.cubic_spline_traj_callback)
        self.mpc_obs_sub = rospy.Subscriber('/humanoid_mpc_observation', mpc_observation, self.mpc_obs_callback)

        """ arm traj pub """
        self.kuavo_arm_traj_pub = rospy.Publisher('/kuavo_arm_traj', JointState, queue_size=10)
        
        """ Variables """
        self.joint_state = JointState()
        self._flag_recv_traj = False
        self._flag_planing = False
        self.cubicspline_arm_traj_state = planArmState()
        self.cubicspline_arm_traj_state.is_finished = False
        self.cubicspline_arm_traj_state.progress = 0
        self._current_arm_joint_state = []

        """ Thread """
        self.running = True
        self.publish_thread = threading.Thread(target=self.publish_loop)
        self.publish_thread.start()

    def current_arm_joint_state(self):
        return self._current_arm_joint_state
    def mpc_obs_callback(self, msg):
        self._current_arm_joint_state = msg.state.value[24:]
        self._current_arm_joint_state = [round(pos, 2) for pos in self._current_arm_joint_state]

    def wait_finish(self, timeout=40) -> bool:
        """ 等待轨迹规划完成
        :param timeout: 超时时间，单位秒，默认40秒
        :return: bool
        """
        start_time = rospy.Time.now().to_sec()
        while not rospy.is_shutdown():
            if self.cubicspline_arm_traj_state.is_finished:
                rospy.loginfo(f"Trajectory execute finished, time cost: {rospy.Time.now().to_sec() - start_time:.2f}")
                return True
            if rospy.Time.now().to_sec() - start_time > timeout:
                return False
            rospy.sleep(0.1)  # 防止CPU占用过高
    def cubic_spline_traj_callback(self, msg):
        if len(msg.points) == 0:
            return
        point = msg.points[0]
        self.joint_state.name = [
            "l_arm_pitch",
            "l_arm_roll",
            "l_arm_yaw",
            "l_forearm_pitch",
            "l_hand_yaw",
            "l_hand_pitch",
            "l_hand_roll",
            "r_arm_pitch",
            "r_arm_roll",
            "r_arm_yaw",
            "r_forearm_pitch",
            "r_hand_yaw",
            "r_hand_pitch",
            "r_hand_roll",
        ]

        # print("CubicSpline Trajectory Received")

        self.joint_state.position = [math.degrees(pos) for pos in point.positions[:14]]
        self.joint_state.velocity = [math.degrees(vel) for vel in point.velocities[:14]]
        self.joint_state.effort = [0] * 14

    def cubicspline_traj_callback(self, msg):
        """ 轨迹规划状态回调函数
        :param msg: planArmState
        """
        # print(msg)
        if not self._flag_planing:
            return
        
        self.cubicspline_arm_traj_state = msg
        
    def plan_arm_traj_cubicspline(self, times:list, l_r_joint_positions:list, timeout=40):
        rospy.wait_for_service('/cubic_spline/plan_arm_trajectory')
        request = planArmTrajectoryCubicSplineRequest()
        joint_trajectory = JointTrajectory()
        for i in range(len(times)):
            joint_trajectory.points.append(JointTrajectoryPoint())
            joint_trajectory.points[-1].positions = l_r_joint_positions[i]
            joint_trajectory.points[-1].time_from_start = rospy.Duration(times[i])
        request.joint_trajectory = joint_trajectory
        response = self._srv_plan_arm_traj_cubicspline(request)
        if response.success:
            self.cubicspline_arm_traj_state.is_finished = False
            self.cubicspline_arm_traj_state.progress = 0
            print("Trajectory planning success")
        time.sleep(0.5)
        self._flag_planing = True
        self.wait_finish(timeout=timeout)  
        self._flag_planing = False
        return response.success
    
    def publish_loop(self):
        rate = 1000
        while not rospy.is_shutdown() and self.running:
            try:
                if len(self.joint_state.position) == 0:
                    continue
                if not self._flag_planing:
                    continue
                self.kuavo_arm_traj_pub.publish(self.joint_state)
            except Exception as e:
                rospy.logerr(f"Failed to publish arm trajectory: {e}")
            except KeyboardInterrupt:
                break
            time.sleep(1.0 / rate)

    def stop(self):
        self.running = False
        self.publish_thread.join()