#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
from kuavo_msgs.msg import gaitTimeName
from std_msgs.msg import Bool

class CmdPoseTester:
    def __init__(self):
        # 1. 初始化ROS节点
        rospy.init_node('cmd_pose_test_node', anonymous=True)

        # 2. 创建Publisher和Subscriber
        self.pub_cmd_pose_world = rospy.Publisher("/cmd_pose_world", Twist, queue_size=1)
        self.pub_cmd_pose = rospy.Publisher("/cmd_pose", Twist, queue_size=1)

        # 订阅 /humanoid_mpc_gait_time_name 用于检测 gait_name
        rospy.Subscriber("/humanoid_mpc_gait_time_name", gaitTimeName, self.gait_time_name_callback)

        # 订阅 /humanoid/single_step_mode 用于检测单步模式
        rospy.Subscriber("/humanoid/single_step_mode", Bool, self.single_step_mode_callback)

        # 状态变量
        self.current_gait_name = None
        self.single_step_mode = False  # 默认为False

        # 等待一下，确保Publisher和Subscriber都已注册
        rospy.sleep(1.0)
        rospy.loginfo("CmdPoseTester initialized successfully.")

    def gait_time_name_callback(self, msg):
        """
        订阅 /humanoid_mpc_gait_time_name 的回调函数
        用于更新当前机器人的 gait_name 状态。
        """
        self.current_gait_name = msg.gait_name

    def single_step_mode_callback(self, msg):
        """
        订阅 /humanoid/single_step_mode 的回调函数
        用于更新当前单步模式的状态（True/False）。
        """
        self.single_step_mode = msg.data

    def wait_for_command_execution(self, timeout=5000.0):
        """
        等待命令执行完成的判断逻辑（两种情况都需要持续0.5s）：
          情况A：gait_name == 'stance' 连续保持0.5s
          情况B：single_step_mode == True 且 gait_name == 'custom_gait' 连续保持0.5s

        在 timeout 时间内，如果任意一个情况连续保持0.5s，即视为执行完成，返回 True；
        如果在 timeout 时间内都没满足，则返回 False。

        注意：执行完成或超时后，会将 self.current_gait_name = None 置空，避免干扰后续判断。
        """
        start_time = rospy.Time.now().to_sec()
        rate = rospy.Rate(50)  # 检测频率（可根据需要调整）

        # 用于记录两个情况开始计时的时刻（None表示尚未开始计时或已被打断）
        stance_start_time = None
        custom_start_time = None

        while not rospy.is_shutdown():
            now = rospy.Time.now().to_sec()
            elapsed = now - start_time

            # 如果超时，则返回False
            if elapsed > timeout:
                rospy.logwarn("Wait for command execution timed out.")
                # 重置 gait_name 避免影响下一个测试
                self.current_gait_name = None
                return False

            # 1) 检查情况A：gait_name == "stance"
            if self.current_gait_name == "stance":
                if stance_start_time is None:
                    stance_start_time = now  # 开始计时
                else:
                    # 检查是否已连续保持0.5s
                    if (now - stance_start_time) >= 0.5:
                        rospy.loginfo("Robot has been in 'stance' for >= 0.5s. Command done.")
                        self.current_gait_name = None  # 重置，避免干扰下一个指令
                        return True
            else:
                # 如果不再是 stance，则重置计时
                stance_start_time = None

            # 2) 检查情况B：single_step_mode == False 且 gait_name == "custom_gait"
            if not self.single_step_mode and self.current_gait_name == "custom_gait":
                if custom_start_time is None:
                    custom_start_time = now  # 开始计时
                else:
                    # 检查是否已连续保持0.5s
                    if (now - custom_start_time) >= 0.5:
                        rospy.loginfo("Detected single_step_mode=True & gait_name='custom_gait' for >= 0.5s. Command done.")
                        self.current_gait_name = None
                        return True
            else:
                # 如果不再同时满足 single_step_mode=False + gait_name='custom_gait'，则重置计时
                custom_start_time = None

            rate.sleep()

        # 如果循环因节点关闭而退出，也返回False
        return False

    def publish_cmd_pose_world(self, linear_x=0.0, linear_y=0.0, linear_z=0.0,
                               angular_x=0.0, angular_y=0.0, angular_z=0.0):
        """
        发布 Twist 消息到 /cmd_pose_world 话题。
        """
        twist_msg = Twist()
        twist_msg.linear.x = linear_x
        twist_msg.linear.y = linear_y
        twist_msg.linear.z = linear_z
        twist_msg.angular.x = angular_x
        twist_msg.angular.y = angular_y
        twist_msg.angular.z = angular_z

        self.pub_cmd_pose_world.publish(twist_msg)
        rospy.loginfo(
            f"Published /cmd_pose_world: "
            f"linear=({linear_x}, {linear_y}, {linear_z}), "
            f"angular=({angular_x}, {angular_y}, {angular_z})"
        )
    
    def publish_cmd_pose(self, linear_x=0.0, linear_y=0.0, linear_z=0.0,
                        angular_x=0.0, angular_y=0.0, angular_z=0.0):
        """
        发布 Twist 消息到 /cmd_pose 话题。
        """
        twist_msg = Twist()
        twist_msg.linear.x = linear_x
        twist_msg.linear.y = linear_y
        twist_msg.linear.z = linear_z
        twist_msg.angular.x = angular_x
        twist_msg.angular.y = angular_y
        twist_msg.angular.z = angular_z

        self.pub_cmd_pose.publish(twist_msg)
        rospy.loginfo(
            f"Published /cmd_pose: "
            f"linear=({linear_x}, {linear_y}, {linear_z}), "
            f"angular=({angular_x}, {angular_y}, {angular_z})"
        )

    # -------------------------------------------------
    # 阶段检测：condition 持续 check_time 秒
    # -------------------------------------------------
   # -------------------稳定检测函数-------------------
    def wait_phase_condition(self, condition_func, phase_name, check_time=0.1, timeout=0.2):
        """
        在 [timeout] 秒内，检测 condition_func() 是否能连续保持 True 达 [check_time] 秒。
        如果达成则打印 "=== Entered {phase_name} ===" 并返回 True；否则返回 False。
        中途断开则计时清零。
        """
        start_time = rospy.Time.now().to_sec()
        rate = rospy.Rate(50)
        hold_start = None

        while (rospy.Time.now().to_sec() - start_time) < timeout and not rospy.is_shutdown():
            if condition_func():
                if hold_start is None:
                    hold_start = rospy.Time.now().to_sec()
                else:
                    if (rospy.Time.now().to_sec() - hold_start) >= check_time:
                        rospy.loginfo(f"=== Entered {phase_name} ===")
                        return True
            else:
                hold_start = None
            rate.sleep()

        return False

    def check_phase_condition_stable(self, condition_func, check_name,
                                     check_time=0.1, timeout=0.2):
        """
        类似 wait_phase_condition，但不打印 "Entered..."。
        主要用于“跳过”判断时的短暂稳定检测：确认后续阶段的条件确实保持了 0.1s。
        """
        start_time = rospy.Time.now().to_sec()
        rate = rospy.Rate(50)
        hold_start = None

        while (rospy.Time.now().to_sec() - start_time) < timeout and not rospy.is_shutdown():
            if condition_func():
                if hold_start is None:
                    hold_start = rospy.Time.now().to_sec()
                else:
                    if (rospy.Time.now().to_sec() - hold_start) >= check_time:
                        # 稳定检测成功
                        # rospy.loginfo(f"[SkipCheck] {check_name} stable for {check_time}s.")
                        return True
            else:
                hold_start = None
            rate.sleep()

        return False

    # -------------------阶段检测(可跳过单步)-------------------
    def detect_4_phases(self, overall_timeout=20.0, interrupt_phase=None, extra_cmd_pose=None):
        """
        依次检测4阶段：
          Phase1: SingleStep (skipable=True)
          Phase2: Walk       (skipable=False)
          Phase3: SingleStep (skipable=True)
          Phase4: End        (skipable=False)

        仅 Phase1/3 可跳过，Phase2/4 不可跳过。

        跳过逻辑：若后续阶段 j 的条件先“稳定出现”(0.1s)，
                   而且从 current_phase~(j-1) 所有阶段 skipable=True，
                   则跳过这些阶段，直接进入 j。
        """
        start_time = rospy.Time.now().to_sec()
        rate = rospy.Rate(50)

        # 定义 4 阶段
        def phase1_cond():
            return self.single_step_mode and (self.current_gait_name == "custom_gait")

        def phase2_cond():
            return (not self.single_step_mode) and (self.current_gait_name == "walk")

        def phase3_cond():
            return self.single_step_mode and (self.current_gait_name == "custom_gait")

        def phase4_cond():
            return (self.current_gait_name == "stance") or \
                   ((not self.single_step_mode) and (self.current_gait_name == "custom_gait"))

        phases = [
            ("Phase1 (SingleStep)",  phase1_cond, True),
            ("Phase2 (Walk)",        phase2_cond, False),
            ("Phase3 (SingleStep)",  phase3_cond, True),
            ("Phase4 (End)",         phase4_cond, False),
        ]

        current_phase = 0
        total_phases = len(phases)

        while not rospy.is_shutdown():
            now = rospy.Time.now().to_sec()
            if (now - start_time) > overall_timeout:
                rospy.logwarn("detect_4_phases timed out.")
                return False

            if current_phase >= total_phases:
                # 已到 Phase4 之后，结束
                return True

            phase_name, cond_func, skipable = phases[current_phase]

            # -------------------------------------------------------------
            # 1) 检查后续阶段: 若它的条件先“稳定出现”，且中间阶段全可跳过 => 跳过
            #    我们从后往前检查，找到最远能跳到的后续阶段 j
            # -------------------------------------------------------------
            jumped = False
            for j in range(total_phases - 1, current_phase, -1):
                j_name, j_cond, j_skipable = phases[j]
                # 先做一个短暂稳定检测(0.1s)来确认 j_cond 真的满足
                if self.check_phase_condition_stable(j_cond, f"{j_name}(SkipCheck)", 0.1, 0.2):
                    # 如果阶段 j 确认了 “稳定出现”
                    # 再判断从 current_phase 到 j-1 是否全部 skipable
                    can_skip_all = True
                    for skip_idx in range(current_phase, j):
                        _, _, sk = phases[skip_idx]
                        if not sk:  # 有不可跳过的阶段
                            can_skip_all = False
                            break
                    if can_skip_all:
                        # 打印跳过
                        for skip_idx in range(current_phase, j):
                            skip_phase_name, _, _ = phases[skip_idx]
                            if skip_idx == current_phase:
                                rospy.logwarn(f"{skip_phase_name} skipped! (Jump to {j_name})")
                            else:
                                rospy.logwarn(f"{skip_phase_name} also skipped!")
                        current_phase = j
                        jumped = True
                        break  # 跳到 j 后退出 for
            if jumped:
                # 跳过去后，在下一轮循环中会检测 “current_phase=j” 的稳定进入
                rate.sleep()
                continue

            # -------------------------------------------------------------
            # 2) 当前阶段条件是否已出现？
            # -------------------------------------------------------------
            if cond_func():
                # 用 wait_phase_condition 做 0.1s 稳定检测
                stable = self.wait_phase_condition(cond_func, phase_name, 0.1, 0.2)
                if stable:
                    rospy.loginfo(f"Entered {phase_name}, curent_phase={current_phase+1}.")
                    current_phase += 1
                    if interrupt_phase is not None and current_phase == interrupt_phase:
                        rospy.logwarn(f"Interrupting {phase_name}...")
                        self.publish_cmd_pose(*extra_cmd_pose)
                    if current_phase == total_phases:
                        rospy.loginfo("Reached Phase4 => End detection now.")
                        return True
            else:
                # 什么都不做，下一次循环再判断
                pass

            rate.sleep()




    def run_test(self, test_commands ):
        """
        test_commands: 列表，每个元素是一个字典，例如：
          {

            "cmd_pose_world": (lx, ly, lz, ax, ay, az),
            "interrupt_phase": 2,
            "extra_cmd_pose": (lx2, ly2, lz2, ax2, ay2, az2) or None
          }
        """


        rospy.loginfo("Start publishing test commands...")

        for idx, cmd in enumerate(test_commands):
            rospy.loginfo(f"--- Test Command {idx+1}/{len(test_commands)} ---")
            cmd_vals = cmd.get("cmd_pose_world", (0,0,0,0,0,0))
            self.publish_cmd_pose_world(*cmd_vals)

            # 等待机器人满足“stance 连续0.5s”或 “(single_step_mode=True & custom_gait) 连续0.5s”
            # if self.wait_for_command_execution(timeout=100.0):

            interrupt_phase = cmd.get("interrupt_phase", None)
            extra_cmd_pose = cmd.get("extra_cmd_pose", None)


            self.detect_4_phases(overall_timeout=50.0, interrupt_phase=interrupt_phase, extra_cmd_pose=extra_cmd_pose)
            rospy.loginfo(f"Command {idx+1} executed successfully.\n")
            rospy.sleep(0.5)
            
            # else:
            #     rospy.logwarn(f"Command {idx+1} timed out before completion.\n")

        rospy.loginfo("All test commands have been published. Exiting...")

def main():
    # 创建测试类对象并运行测试
    tester = CmdPoseTester()
            # 根据需求添加或修改测试动作
    # test_commands = [
    #     (1.0,  0.0,  0.0,   0.0, 0.0, 0.0),   # 例：不动
    #     (0.2,  0.0,  0.0,   0.0, 0.0, 0.0),   # 例：向前移动
    #     (0.0, -1.2,  0.0,   0.0, 0.0, 0.0),   # 例：向右移动
    #     (0.0,  0.0,  0.0,   0.0, 0.0, 1.0),   # 例：原地旋转
    #     (0.3,  0.1,  0.0,   0.0, 0.0, 0.5),   # 更多复合动作
    # ]

    foward_commands1 = (1.0, 1.2, 0.0, 0.0, 0.0, 0.0)
    backward_commands1 = (-1.0, -1.5, 0.0, 0.0, 0.0, 0.0)

    foward_commands = foward_commands1
    backward_commands = backward_commands1

    test_commands = [
        {

            "cmd_pose_world": foward_commands,
            "interrupt_phase": None,  # 不打断
            "extra_cmd_pose": (1.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        },
        {

            "cmd_pose_world": (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
            "interrupt_phase": None,  # 若进入Phase1时就打断
            "extra_cmd_pose": (1.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        },
        {

            "cmd_pose_world": backward_commands,
            "interrupt_phase": None,  # 不打断
            "extra_cmd_pose": (1.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        },
        {

            "cmd_pose_world": (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
            "interrupt_phase": None,  # 若进入Phase1时就打断
            "extra_cmd_pose": (1.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        },


        {

            "cmd_pose_world": foward_commands,
            "interrupt_phase": 1,  # 1阶段打断
            "extra_cmd_pose": (1.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        },
        {

            "cmd_pose_world": (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
            "interrupt_phase": None,  # 若进入Phase1时就打断
            "extra_cmd_pose": (1.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        },
        {

            "cmd_pose_world": backward_commands,
            "interrupt_phase": 1,  # 1阶段打断
            "extra_cmd_pose": (1.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        },
        {

            "cmd_pose_world": (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
            "interrupt_phase": None,  # 若进入Phase1时就打断
            "extra_cmd_pose": (1.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        },


        {

            "cmd_pose_world": foward_commands,
            "interrupt_phase": 2,  # 2阶段打断
            "extra_cmd_pose": (1.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        },
        {

            "cmd_pose_world": (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
            "interrupt_phase": None,  # 若进入Phase1时就打断
            "extra_cmd_pose": (1.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        },
        {

            "cmd_pose_world": backward_commands,
            "interrupt_phase": 2,  # 2阶段打断
            "extra_cmd_pose": (1.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        },
        {

            "cmd_pose_world": (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
            "interrupt_phase": None,  # 若进入Phase1时就打断
            "extra_cmd_pose": (1.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        },



        {

            "cmd_pose_world": foward_commands,
            "interrupt_phase": 3,  # 3阶段打断
            "extra_cmd_pose": (1.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        },
        {

            "cmd_pose_world": (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
            "interrupt_phase": None,  # 若进入Phase1时就打断
            "extra_cmd_pose": (1.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        },
        {

            "cmd_pose_world": backward_commands,
            "interrupt_phase": 3,  # 3阶段打断
            "extra_cmd_pose": (1.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        },
        {

            "cmd_pose_world": (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
            "interrupt_phase": None,  # 若进入Phase1时就打断
            "extra_cmd_pose": (1.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        },

        # {
        #     "enable_cmd_pose": False,  # 不发布主cmd_pose
        #     "cmd_pose_world": (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),  # 不会用
        #     "interrupt_phase": 2,
        #     "extra_cmd_pose": (0.0, -0.3, 0.0, 0.0, 0.0, 0.0)
        # },
        # {

        #     "cmd_pose_world": (0.0, 0.0, 0.0, 0.0, 0.0, 1.0),
        #     "interrupt_phase": None,  # 直接在进入Phase4时结束
        #     "extra_cmd_pose": None
        # },
    ]


    tester.run_test(test_commands)

if __name__ == "__main__":
    main()
