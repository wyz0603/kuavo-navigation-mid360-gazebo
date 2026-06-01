import numpy as np
from functools import partial
from pydrake.all import (InverseKinematics, Solve, RotationMatrix, RollPitchYaw, SnoptSolver,InitializeAutoDiff,JacobianWrtVariable,
                         ExtractGradient, ExtractValue, ComPositionConstraint, CentroidalMomentumConstraint)


from scipy.spatial.transform import Rotation as R

class TorsoIK:
    def __init__(self, plant, frame_name_list, constraint_tol=1.0e-8, solver_tol=1.0e-6):
        self.__plant = plant
        self.__plant_context = self.__plant.CreateDefaultContext()
        self.__constraint_tol = constraint_tol
        self.__solver_tol = solver_tol
        self.__frames = [self.__plant.GetFrameByName(name) for name in frame_name_list]
        self.__normal_q = np.array([-0.3928, 0.2608, 0.0143, -0.3927, 0.2970, -0.8364, 0.0589,
                             -0.3678, 0.1778, 0.00284, -0.4253, 0.2398, -0.9134, -0.0256])
        # self.__normal_q = np.array([-0.3928, 0.2608, 0.0143, -0.3927, 0.2970, -0.8364, 0.0589,
        #                      -0.3928, -0.2608, -0.0143, -0.3927, -0.2970, -0.8364, -0.0589])

    def set_arm_normal_q(self, q:list):
        self.__normal_q = np.array(q)
        
    def solve(self, pose_list, q0=[]):
        self.__IK = InverseKinematics(plant=self.__plant, with_joint_limits=True)
        for i, frame in enumerate(self.__frames):
            if not pose_list[i][0] is None:
                self.__IK.AddOrientationConstraint(self.__plant.world_frame(), RotationMatrix(RollPitchYaw(pose_list[i][0])),
                                            frame, RotationMatrix(RollPitchYaw(0, 0, 0)),
                                            self.__constraint_tol)
            if not pose_list[i][1] is None:
                self.__IK.AddPositionConstraint(frameB=frame,
                                                p_BQ=np.zeros(3),
                                                frameA=self.__plant.world_frame(),
                                                p_AQ_lower=np.array(pose_list[i][1])-self.__constraint_tol,
                                                p_AQ_upper=np.array(pose_list[i][1])+self.__constraint_tol)
        W = np.diag([0.1]*14)
        self.__IK.prog().AddQuadraticErrorCost(W, self.__normal_q, self.__IK.q()[-14:])
        result = Solve(self.__IK.prog(), q0)
        if result.is_success():
            return [True, result.GetSolution()]
        return [False, []]
