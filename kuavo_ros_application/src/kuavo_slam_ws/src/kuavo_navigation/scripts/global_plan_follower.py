#!/usr/bin/env python3
"""Pure-pursuit follower for move_base's global plan.

Architecture:
  global plan (move_base GlobalPlanner) ---> this node ---> /cmd_vel ---> robot
                                              |
                                              +-- monitors local_costmap on the
                                                  upcoming plan segment; if any
                                                  cell ahead is LETHAL the node
                                                  asks move_base to clear and
                                                  re-plan (instead of letting
                                                  MPC try to squeeze through).

The local MPC keeps running (for visualization / diagnostics) but its
/cmd_vel is remapped to /cmd_vel_mpc_unused in the launch so the robot
listens to THIS node.

Why this design:
  - MPC's smoothed trajectory deviates from the global planner's intended
    corridor and ends up hugging inflation zones / getting stuck.
  - The global planner already routes around obstacles; pure pursuit just
    faithfully executes its waypoints.
  - Reactive replanning (rather than MPC re-solving) is more responsive when
    new obstacles appear ahead.
"""

import math
import threading

import rospy
import tf2_ros
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path, OccupancyGrid
from std_srvs.srv import Empty
from actionlib_msgs.msg import GoalID, GoalStatusArray
from move_base_msgs.msg import MoveBaseActionGoal


LETHAL_THRESHOLD = 90    # costmap cell value above which we treat as blocking
COSTMAP_UNKNOWN = -1


def quat_to_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class GlobalPlanFollower:
    def __init__(self):
        # --- params ----------------------------------------------------------
        self.lookahead_dist     = rospy.get_param("~lookahead_dist", 0.6)
        self.linear_vel         = rospy.get_param("~linear_vel", 0.35)
        self.max_angular_vel    = rospy.get_param("~max_angular_vel", 0.8)
        self.goal_tolerance_xy  = rospy.get_param("~goal_tolerance_xy", 0.25)
        self.goal_tolerance_yaw = rospy.get_param("~goal_tolerance_yaw", 0.20)
        self.blocked_check_ahead = rospy.get_param("~blocked_check_ahead", 2.5)  # m of plan to scan
        self.blocked_check_step  = rospy.get_param("~blocked_check_step", 0.10)
        self.replan_cooldown     = rospy.get_param("~replan_cooldown", 3.0)      # seconds between replan triggers
        self.global_frame        = rospy.get_param("~global_frame", "map")
        self.base_frame          = rospy.get_param("~base_frame", "lio_base_link")
        self.control_rate        = rospy.get_param("~control_rate", 20.0)        # Hz
        self.plan_topic          = rospy.get_param("~plan_topic", "/move_base/GlobalPlanner/plan")
        self.costmap_topic       = rospy.get_param("~costmap_topic", "/move_base/local_costmap/costmap")
        self.cmd_vel_topic       = rospy.get_param("~cmd_vel_topic", "/cmd_vel")

        # --- state -----------------------------------------------------------
        self.lock = threading.Lock()
        self.plan_poses = []           # list[(x, y, yaw)] in global_frame
        self.local_costmap = None      # nav_msgs/OccupancyGrid
        self.current_goal = None       # geometry_msgs/PoseStamped, latest active goal
        self.last_replan_time = rospy.Time(0)
        self.last_goal_id_seq = 0

        # --- I/O -------------------------------------------------------------
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.cmd_pub = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=1)
        self.goal_pub = rospy.Publisher("/move_base/goal", MoveBaseActionGoal,
                                        queue_size=1)
        self.cancel_pub = rospy.Publisher("/move_base/cancel", GoalID,
                                          queue_size=1)

        rospy.Subscriber(self.plan_topic, Path, self.cb_plan, queue_size=1)
        rospy.Subscriber(self.costmap_topic, OccupancyGrid, self.cb_costmap, queue_size=1)
        rospy.Subscriber("/move_base/current_goal", PoseStamped, self.cb_current_goal, queue_size=1)

        rospy.loginfo("global_plan_follower: ready. plan=%s costmap=%s cmd_vel=%s",
                      self.plan_topic, self.costmap_topic, self.cmd_vel_topic)

    # ------------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------------
    def cb_plan(self, msg):
        with self.lock:
            self.plan_poses = [(p.pose.position.x,
                                p.pose.position.y,
                                quat_to_yaw(p.pose.orientation))
                               for p in msg.poses]

    def cb_costmap(self, msg):
        with self.lock:
            self.local_costmap = msg

    def cb_current_goal(self, msg):
        with self.lock:
            self.current_goal = msg

    # ------------------------------------------------------------------------
    # TF helper
    # ------------------------------------------------------------------------
    def get_robot_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform(self.global_frame, self.base_frame,
                                                 rospy.Time(0), rospy.Duration(0.1))
            x = tf.transform.translation.x
            y = tf.transform.translation.y
            yaw = quat_to_yaw(tf.transform.rotation)
            return x, y, yaw
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException,
                tf2_ros.ConnectivityException):
            return None

    # ------------------------------------------------------------------------
    # Costmap lookup
    # ------------------------------------------------------------------------
    def costmap_value_at(self, x, y, cm):
        """Return cell value at world (x, y) in the costmap frame, or None."""
        if cm is None:
            return None
        # Costmap is in cm.header.frame_id (typically `odom` for local). The
        # global plan is in `map`. We must transform (x, y) from global_frame
        # into the costmap's frame.
        try:
            tf = self.tf_buffer.lookup_transform(cm.header.frame_id, self.global_frame,
                                                 rospy.Time(0), rospy.Duration(0.05))
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException,
                tf2_ros.ConnectivityException):
            return None
        # Apply 2D rotation + translation
        tx = tf.transform.translation.x
        ty = tf.transform.translation.y
        yaw = quat_to_yaw(tf.transform.rotation)
        cos_y, sin_y = math.cos(yaw), math.sin(yaw)
        wx = cos_y * x - sin_y * y + tx
        wy = sin_y * x + cos_y * y + ty
        # Now (wx, wy) is in costmap frame
        mx = int((wx - cm.info.origin.position.x) / cm.info.resolution)
        my = int((wy - cm.info.origin.position.y) / cm.info.resolution)
        if mx < 0 or my < 0 or mx >= cm.info.width or my >= cm.info.height:
            return None
        idx = my * cm.info.width + mx
        return cm.data[idx]

    # ------------------------------------------------------------------------
    # Plan helpers
    # ------------------------------------------------------------------------
    def find_lookahead(self, rx, ry, plan):
        """Return (lx, ly, idx) - point on plan at lookahead_dist from (rx, ry)
        and its index, or (None, None, None) if no plan."""
        if not plan:
            return None, None, None
        # Find nearest point on plan
        best_i = 0
        best_d2 = float("inf")
        for i, (x, y, _) in enumerate(plan):
            d2 = (x - rx) ** 2 + (y - ry) ** 2
            if d2 < best_d2:
                best_d2 = d2
                best_i = i
        # Walk forward until we reach lookahead distance
        accum = 0.0
        i = best_i
        while i < len(plan) - 1:
            ax, ay, _ = plan[i]
            bx, by, _ = plan[i + 1]
            seg = math.hypot(bx - ax, by - ay)
            if accum + seg >= self.lookahead_dist:
                t = (self.lookahead_dist - accum) / max(seg, 1e-6)
                return ax + t * (bx - ax), ay + t * (by - ay), i + 1
            accum += seg
            i += 1
        # Plan too short - use last point
        return plan[-1][0], plan[-1][1], len(plan) - 1

    def is_plan_blocked(self, rx, ry, plan):
        """Walk the plan from current robot position forward by
        `blocked_check_ahead` meters; return True if any sampled point lands on
        a LETHAL costmap cell."""
        with self.lock:
            cm = self.local_costmap
        if cm is None or not plan:
            return False
        # Same nearest-index logic as find_lookahead
        best_i = 0
        best_d2 = float("inf")
        for i, (x, y, _) in enumerate(plan):
            d2 = (x - rx) ** 2 + (y - ry) ** 2
            if d2 < best_d2:
                best_d2 = d2
                best_i = i
        # Sample along plan
        accum = 0.0
        i = best_i
        while i < len(plan) - 1 and accum < self.blocked_check_ahead:
            ax, ay, _ = plan[i]
            bx, by, _ = plan[i + 1]
            seg = math.hypot(bx - ax, by - ay)
            steps = max(1, int(seg / self.blocked_check_step))
            for s in range(steps + 1):
                t = s / steps
                wx = ax + t * (bx - ax)
                wy = ay + t * (by - ay)
                val = self.costmap_value_at(wx, wy, cm)
                if val is not None and val != COSTMAP_UNKNOWN and val >= LETHAL_THRESHOLD:
                    return True
                if accum + t * seg >= self.blocked_check_ahead:
                    return False
            accum += seg
            i += 1
        return False

    # ------------------------------------------------------------------------
    # Recovery: ask move_base to clear costmaps and re-plan
    # ------------------------------------------------------------------------
    def trigger_replan(self):
        now = rospy.Time.now()
        if (now - self.last_replan_time).to_sec() < self.replan_cooldown:
            return
        self.last_replan_time = now

        rospy.logwarn("global_plan_follower: plan blocked ahead, asking move_base to clear and replan")

        # Clear both costmaps
        try:
            rospy.wait_for_service("/move_base/clear_costmaps", timeout=1.0)
            rospy.ServiceProxy("/move_base/clear_costmaps", Empty)()
        except (rospy.ROSException, rospy.ServiceException) as e:
            rospy.logwarn("global_plan_follower: clear_costmaps failed: %s", e)

        # Re-publish the current goal to force move_base to re-plan from scratch
        with self.lock:
            goal = self.current_goal
        if goal is None:
            return
        action_goal = MoveBaseActionGoal()
        action_goal.header.stamp = now
        self.last_goal_id_seq += 1
        action_goal.goal_id.stamp = now
        action_goal.goal_id.id = "global_plan_follower_%d" % self.last_goal_id_seq
        action_goal.goal.target_pose = goal
        self.goal_pub.publish(action_goal)

    # ------------------------------------------------------------------------
    # Main control loop
    # ------------------------------------------------------------------------
    def run(self):
        rate = rospy.Rate(self.control_rate)
        while not rospy.is_shutdown():
            rate.sleep()

            pose = self.get_robot_pose()
            if pose is None:
                continue
            rx, ry, ryaw = pose

            with self.lock:
                plan = list(self.plan_poses)
                goal = self.current_goal

            if not plan or goal is None:
                # No active goal or plan, sit still.
                self.cmd_pub.publish(Twist())
                continue

            # Goal reached?
            gx = goal.pose.position.x
            gy = goal.pose.position.y
            gyaw = quat_to_yaw(goal.pose.orientation)
            d_goal = math.hypot(gx - rx, gy - ry)
            if d_goal < self.goal_tolerance_xy:
                yaw_err = self._wrap_angle(gyaw - ryaw)
                if abs(yaw_err) < self.goal_tolerance_yaw:
                    self.cmd_pub.publish(Twist())   # at goal, stop
                    continue
                # Spin in place to align final yaw
                cmd = Twist()
                cmd.angular.z = max(-self.max_angular_vel,
                                    min(self.max_angular_vel,
                                        1.5 * yaw_err))
                self.cmd_pub.publish(cmd)
                continue

            # Plan blocked? -> ask for replan, do not move
            if self.is_plan_blocked(rx, ry, plan):
                self.cmd_pub.publish(Twist())
                self.trigger_replan()
                continue

            # Pure pursuit
            lx, ly, _ = self.find_lookahead(rx, ry, plan)
            if lx is None:
                self.cmd_pub.publish(Twist())
                continue
            target_yaw = math.atan2(ly - ry, lx - rx)
            yaw_err = self._wrap_angle(target_yaw - ryaw)

            cmd = Twist()
            # Slow down when sharply turning so feet don't go crazy
            cmd.linear.x = self.linear_vel * max(0.2, math.cos(yaw_err))
            cmd.angular.z = max(-self.max_angular_vel,
                                min(self.max_angular_vel,
                                    2.0 * yaw_err))
            self.cmd_pub.publish(cmd)

    @staticmethod
    def _wrap_angle(a):
        while a > math.pi:
            a -= 2.0 * math.pi
        while a < -math.pi:
            a += 2.0 * math.pi
        return a


def main():
    rospy.init_node("global_plan_follower")
    node = GlobalPlanFollower()
    node.run()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
