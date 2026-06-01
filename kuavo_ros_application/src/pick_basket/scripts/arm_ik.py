import os
import time
import math
import numpy as np
import pydrake

from pydrake.all import (
    AddMultibodyPlantSceneGraph, DiagramBuilder, Parser, PiecewisePolynomial
)
from pydrake.all import StartMeshcat, AddMultibodyPlantSceneGraph, MeshcatVisualizer
from pydrake.math import RotationMatrix


from common.IK import *
from common.utils import *
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import Pose

script_dir = os.path.dirname(os.path.abspath(__file__))

class ArmIk:
    def __init__(self, model_file, end_frames_name, meshcat):
        builder = DiagramBuilder()
        self.__plant, scene_graph = AddMultibodyPlantSceneGraph(builder, 1e-3)
        parser = Parser(self.__plant)
        parser.AddModels(model_file)
        self.__plant.Finalize()

        self.__visualizer = MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat)
        self.__diagram = builder.Build()
        self.__diagram_context = self.__diagram.CreateDefaultContext()

        self.__plant_context = self.__plant.GetMyContextFromRoot(self.__diagram_context)
        self.__q0 = self.__plant.GetPositions(self.__plant_context)
        self.__curr_q =  self.__q0
        self.__v0 = self.__plant.GetVelocities(self.__plant_context)
        self.__r0 = self.__plant.CalcCenterOfMassPositionInWorld(self.__plant_context)
        
        self.__plant.SetPositions(self.__plant_context, self.__q0)
        self.__base_link_name = end_frames_name[0]
        self.__left_eef_name = end_frames_name[1]
        self.__right_eef_name = end_frames_name[2]
        
        self.__torso_yaw_rad = 0      
        self.__torso_height = 0
        self.__IK = TorsoIK(self.__plant, end_frames_name, 1e-4, 1e-4)

        # print("num_positions:", self.__plant.num_positions())
        # print("GetPositionNames:", self.__plant.GetPositionNames())    

    def q0(self):
        return self.__q0
    def set_arm_joint_state(self, joint_state:list):
        self.__curr_q[7:21] = joint_state # rad
        # TODO remove
        # self.__IK.set_arm_normal_q(self.__curr_q[7:21])
    def get_current_arm_q(self):
        return self.__curr_q[-14:]
    def get_current_r_arm_q(self):
        arm_q = self.get_current_arm_q()
        return arm_q[-7:]
    def get_current_l_arm_q(self):
        arm_q = self.get_current_arm_q()
        return arm_q[:7]
    def set_curr_q(self, q):
        self.__curr_q = q
    def curr_q(self):
        return self.__curr_q    
    
    def init_state(self, torso_yaw_deg:float, torso_height:float, joint_state:list):
        self.__torso_yaw_rad = math.radians(torso_yaw_deg)       
        self.__torso_height = torso_height       
        self.__q0[6] = self.__torso_height

        # left arm: [7:13], right arm: [14:31]
        self.__q0[7:21] = joint_state # rad
        self.__curr_q = self.__q0

    def computeArmIK(self, start_q, l_ps:Pose, r_ps:Pose):
        l_hand_pose = None
        l_hand_RPY = None
        r_hand_pose = None
        r_hand_RPY = None

        if l_ps is not None:
            l_hand_pose = [l_ps.position.x, l_ps.position.y, l_ps.position.z]
            x = l_ps.orientation.x
            y = l_ps.orientation.y
            z = l_ps.orientation.z
            w = l_ps.orientation.w
            l_hand_RPY = R.from_quat([x,y,z,w]).as_euler('xyz', degrees=False)
        if r_ps is not None:
            r_hand_pose = [r_ps.position.x, r_ps.position.y, r_ps.position.z]
            x = r_ps.orientation.x
            y = r_ps.orientation.y
            z = r_ps.orientation.z
            w = r_ps.orientation.w
            r_hand_RPY = R.from_quat([x,y,z,w]).as_euler('xyz', degrees=False)

        q = self.computeIK(start_q, l_hand_pose, r_hand_pose, l_hand_RPY, r_hand_RPY)
        if q is not None:
            return q[-14:]
        return None
    def computeIK(self, start_q, l_hand_pose, r_hand_pose, l_hand_RPY=None, r_hand_RPY=None):
            torsoR = [0.0, self.__torso_yaw_rad, 0.0]
            r = [0.0, 0.0, self.__torso_height]
            
            pose_list = [
                [torsoR, r],               # torso
                [l_hand_RPY, l_hand_pose], # l_hand_pose
                [r_hand_RPY, r_hand_pose], # r_hand_pose 
            ]
            is_success, q = self.__IK.solve(pose_list, start_q)
            if not is_success:
                # print(f"left hand: rpy:{pose_list[1][0]}, xyz:{pose_list[1][1]}")
                # print(f"right hand: rpy:{pose_list[2][0]}, xyz:{pose_list[2][1]}")
                return None
            else:
                return q 

    def start_recording(self):
        self.__visualizer.StartRecording()

    def stop_andpublish_recording(self):
        self.__visualizer.StopRecording()
        self.__visualizer.PublishRecording()

    def visualize_animation(self, q_list, start_time=0.0, duration=1.1):
        t_sol = np.arange(start_time, start_time+duration, 1)  
        q_sol = np.array(q_list).T
        q_pp = PiecewisePolynomial.FirstOrderHold(t_sol, q_sol)
        t0 = t_sol[0]
        tf = t_sol[-1]
        t = t0
        while t < tf:
            q = q_pp.value(t)
            self.__plant.SetPositions(self.__plant_context, q)
            self.__diagram_context.SetTime(t)
            self.__diagram.ForcedPublish(self.__diagram_context)
            t += 0.01

        time.sleep(0.1)
    
    def left_hand_pose(self, q):
        self.__plant.SetPositions(self.__plant_context, q)
        l_hand_in_base = self.__plant.GetFrameByName(self.__left_eef_name).CalcPose(self.__plant_context, self.__plant.GetFrameByName(self.__base_link_name))
        return l_hand_in_base
    
    def right_hand_pose(self, q):
        self.__plant.SetPositions(self.__plant_context, q)
        r_hand_in_base = self.__plant.GetFrameByName(self.__right_eef_name).CalcPose(self.__plant_context, self.__plant.GetFrameByName(self.__base_link_name))
        return r_hand_in_base

def rad_to_angle(rad_list: list) -> list:
    """ 弧度转变为角度 """
    return (np.array(rad_list)/np.pi*180).tolist()


def angle_to_rad(angle_list: list) -> list:
    """ 角度转变为弧度 """
    return (np.array(angle_list)/180*np.pi).tolist()

if __name__ == "__main__":

    base_link_name = 'torso'
    end_frames_name = [base_link_name,'l_hand_end_virtual', 'r_hand_end_virtual']
    model_file_path= script_dir + "/../../../ros_robotModel/biped_s3/urdf/biped_s3_arm.urdf"

    meshcat = StartMeshcat()
    arm_ik = ArmIk(model_file_path, end_frames_name, meshcat)

    right_hand_joint_state = [-0.8626515552558344, -0.006376053566765065,0.001443316443879032,
                              -0.5258335266847197, -0.002796490202113557, -0.3474318375331432,
                              0.17483084841347962]
    arm_joint_state = [0] * 7 + right_hand_joint_state
    
    arm_ik.init_state(0.0, 0.0, arm_joint_state)
    
    curr_q = arm_ik.q0()
   
    q_list = [curr_q]
    last_q = curr_q
    t = 0.0

    for i in range(250):
        l_hand_pose = None
        l_hand_RPY = None
        # r_hand_RPY =[1.30899, 1.30899, 1.30899]
        # print("as_quat:", R.from_euler('xyz', r_hand_RPY).as_quat())
        r_hand_RPY = R.from_quat([-0.0005388071066334781, -0.7904212674887817, 0.00032694187655405566, 0.6125633213777487]).as_euler('xyz', degrees=False)      
        r_hand_pose = [0.24405573596968235943, -0.08740545708279363, 0.08024433670557732]
        
        time_0 = time.time()
        q = arm_ik.computeIK(curr_q, l_hand_pose, r_hand_pose, l_hand_RPY, r_hand_RPY)
        time_cost = time.time() - time_0
        print(f"time cost: {1e3*time_cost:.3f} ms")

        if q is not None:
            q_list.append(q)
            arm_ik.set_curr_q(q)
            # animate trajectory
            arm_ik.visualize_animation([last_q, q], t)
            last_q = q
            t = t + 1.0
            print("right hand q:\n", arm_ik.get_current_r_arm_q())
            print("", rad_to_angle(arm_ik.get_current_r_arm_q()))
        else:
            print(f"Failed to IK in step {i}!")

    arm_ik.stop_andpublish_recording()
    print('Program end, Press Ctrl + C to exit.')
    while True:
        time.sleep(0.01)
