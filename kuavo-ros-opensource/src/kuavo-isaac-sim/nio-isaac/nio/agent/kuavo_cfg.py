# import math
from dataclasses import dataclass
import os 

"""
    - 45 4pro长手index
[2, 3, 7, 8, 12, 13, 16, 17, 20, 21, 24, 25, 26, 27] # 手
[0, 1, 5, 6, 10, 11, 14, 15, 18, 19, 22, 23] # 脚
leg_l1_joint:  0
leg_l2_joint:  5
leg_l3_joint:  10
leg_l4_joint:  14
leg_l5_joint:  18
leg_l6_joint:  22
leg_r1_joint:  1
leg_r2_joint:  6
leg_r3_joint:  11
leg_r4_joint:  15
leg_r5_joint:  19
leg_r6_joint:  23
zarm_l1_joint:  2
zarm_l2_joint:  7
zarm_l3_joint:  12
zarm_l4_joint:  16
zarm_l5_joint:  20
zarm_l6_joint:  24
zarm_l7_joint:  26
zarm_r1_joint:  3
zarm_r2_joint:  8
zarm_r3_joint:  13
zarm_r4_joint:  17
zarm_r5_joint:  21
zarm_r6_joint:  25
zarm_r7_joint:  27
"""

"""
    - 40 4代短手短脚index
[2, 3, 6, 7, 10, 11, 14, 15, 18, 19, 22, 23, 24, 25] # 手
[0, 1, 4, 5, 8, 9, 12, 13, 16, 17, 20, 21] # 脚
leg_l1_joint:  0
leg_l2_joint:  4
leg_l3_joint:  8
leg_l4_joint:  12
leg_l5_joint:  16
leg_l6_joint:  20
leg_r1_joint:  1
leg_r2_joint:  5
leg_r3_joint:  9
leg_r4_joint:  13
leg_r5_joint:  17
leg_r6_joint:  21
zarm_l1_joint:  2
zarm_l2_joint:  6
zarm_l3_joint:  10
zarm_l4_joint:  14
zarm_l5_joint:  18
zarm_l6_joint:  22
zarm_l7_joint:  24
zarm_r1_joint:  3
zarm_r2_joint:  7
zarm_r3_joint:  11
zarm_r4_joint:  15
zarm_r5_joint:  19
zarm_r6_joint:  23
zarm_r7_joint:  25
"""

"""
    - /sensor_data_raw
leg_l1_joint
leg_l2_joint
leg_l3_joint
leg_l4_joint
leg_l5_joint
leg_l6_joint
leg_r1_joint
leg_r2_joint
leg_r3_joint
leg_r4_joint
leg_r5_joint
leg_r6_joint
zarm_l1_joint
zarm_l2_joint
zarm_l3_joint
zarm_l4_joint
zarm_l5_joint
zarm_l6_joint
zarm_l7_joint
zarm_r1_joint
zarm_r2_joint
zarm_r3_joint
zarm_r4_joint
zarm_r5_joint
zarm_r6_joint
zarm_r7_joint

    - /joint_cmd
leg_l1_joint
leg_l2_joint
leg_l3_joint
leg_l4_joint
leg_l5_joint
leg_l6_joint
leg_r1_joint
leg_r2_joint
leg_r3_joint
leg_r4_joint
leg_r5_joint
leg_r6_joint
zarm_l1_joint
zarm_l2_joint
zarm_l3_joint
zarm_l4_joint
zarm_l5_joint
zarm_l6_joint
zarm_l7_joint
zarm_r1_joint
zarm_r2_joint
zarm_r3_joint
zarm_r4_joint
zarm_r5_joint
zarm_r6_joint
zarm_r7_joint
"""

ROBOT_CFG_VERSION = int(os.getenv('ROBOT_VERSION', '45'))  # 45 作为默认值

# 默认为长脚站立的时候的初始角度
leg_initial_pos = [
    0.01867,  # 左腿
    -0.00196,
    -0.43815,
    0.80691,
    -0.31346,
    0.01878,
    -0.01867, # 右腿
    0.00196,
    -0.43815,
    0.80691,
    -0.31346,
    -0.01878,
]

# -- Default joint positions
if ROBOT_CFG_VERSION == 40 or ROBOT_CFG_VERSION == 43:  # 40机器人版本 | 短脚版本
    leg_initial_pos = [
        -0.01867,
        -0.00196,
        -0.43815,
        0.86691,
        -0.31346,
        0.01878,
        0.01868,
        0.00197,
        -0.43815,
        0.86692,
        -0.31347,
        -0.01878,
    ]
# elif ROBOT_CFG_VERSION == 45 or ROBOT_CFG_VERSION == 41 or ROBOT_CFG_VERSION == 42: # 45机器人版本

# 初始手臂位置
arm_initial_pos = [0] * 14
TAU_LEG_FLAG = True
TAU_ARM_FLAG = False

if TAU_LEG_FLAG: # 纯力矩控制模式
    leg_dof_stiffness = {
        "leg_l1_joint": 0,
        "leg_l2_joint": 0,
        "leg_l3_joint": 0,
        "leg_l4_joint": 0,
        "leg_l5_joint": 0, # ankle
        "leg_l6_joint": 0, # ankle
        "leg_r1_joint": 0,
        "leg_r2_joint": 0,
        "leg_r3_joint": 0,
        "leg_r4_joint": 0,
        "leg_r5_joint": 0, # ankle
        "leg_r6_joint": 0, # ankle
    }

    leg_dof_damping = {
        "leg_l1_joint": 0,
        "leg_l2_joint": 0,
        "leg_l3_joint": 0,
        "leg_l4_joint": 0,
        "leg_l5_joint": 0, # ankle
        "leg_l6_joint": 0, # ankle
        "leg_r1_joint": 0,
        "leg_r2_joint": 0,
        "leg_r3_joint": 0,
        "leg_r4_joint": 0,
        "leg_r5_joint": 0, # ankle
        "leg_r6_joint": 0, # ankle
    }
else: # 混合控制模式CSP
    leg_dof_stiffness = {
        "leg_l1_joint": 200,
        "leg_l2_joint": 200,
        "leg_l3_joint": 350,
        "leg_l4_joint": 350,
        "leg_l5_joint": 200, # ankle
        "leg_l6_joint": 200, # ankle
        "leg_r1_joint": 200,
        "leg_r2_joint": 200,
        "leg_r3_joint": 350,
        "leg_r4_joint": 350,
        "leg_r5_joint": 200, # ankle
        "leg_r6_joint": 200, # ankle
    }

    leg_dof_damping = {
        "leg_l1_joint": 30,
        "leg_l2_joint": 30,
        "leg_l3_joint": 30,
        "leg_l4_joint": 30,
        "leg_l5_joint": 10, # ankle
        "leg_l6_joint": 10, # ankle
        "leg_r1_joint": 30,
        "leg_r2_joint": 30,
        "leg_r3_joint": 30,
        "leg_r4_joint": 30,
        "leg_r5_joint": 10, # ankle
        "leg_r6_joint": 10, # ankle
    }

if TAU_ARM_FLAG: # 纯力矩控制模式
# 为手臂和腿部分别创建Kp和Kd字典
    arm_dof_stiffness = {
        "zarm_l1_joint": 0,
        "zarm_l2_joint": 0,
        "zarm_l3_joint": 0,
        "zarm_l4_joint": 0,
        "zarm_l5_joint": 0,
        "zarm_l6_joint": 0,
        "zarm_l7_joint": 0,
        "zarm_r1_joint": 0,
        "zarm_r2_joint": 0,
        "zarm_r3_joint": 0,
        "zarm_r4_joint": 0,
        "zarm_r5_joint": 0,
        "zarm_r6_joint": 0,
        "zarm_r7_joint": 0,
    }

    arm_dof_damping = {
        "zarm_l1_joint": 0,
        "zarm_l2_joint": 0,
        "zarm_l3_joint": 0,
        "zarm_l4_joint": 0,
        "zarm_l5_joint": 0,
        "zarm_l6_joint": 0,
        "zarm_l7_joint": 0,
        "zarm_r1_joint": 0,
        "zarm_r2_joint": 0,
        "zarm_r3_joint": 0,
        "zarm_r4_joint": 0,
        "zarm_r5_joint": 0,
        "zarm_r6_joint": 0,
        "zarm_r7_joint": 0,
    }
else: # 速度控制模式
    # 为手臂和腿部分别创建Kp和Kd字典
    arm_dof_stiffness = {
        "zarm_l1_joint": 0,
        "zarm_l2_joint": 0,
        "zarm_l3_joint": 0,
        "zarm_l4_joint": 0,
        "zarm_l5_joint": 0,
        "zarm_l6_joint": 0,
        "zarm_l7_joint": 0,
        "zarm_r1_joint": 0,
        "zarm_r2_joint": 0,
        "zarm_r3_joint": 0,
        "zarm_r4_joint": 0,
        "zarm_r5_joint": 0,
        "zarm_r6_joint": 0,
        "zarm_r7_joint": 0,
    }

    arm_dof_damping = {
        "zarm_l1_joint": 8,
        "zarm_l2_joint": 8,
        "zarm_l3_joint": 8,
        "zarm_l4_joint": 8,
        "zarm_l5_joint": 8,
        "zarm_l6_joint": 8,
        "zarm_l7_joint": 8,
        "zarm_r1_joint": 8,
        "zarm_r2_joint": 8,
        "zarm_r3_joint": 8,
        "zarm_r4_joint": 8,
        "zarm_r5_joint": 8,
        "zarm_r6_joint": 8,
        "zarm_r7_joint": 8,
    }   

# 添加头部关节配置
head_dof_stiffness = {
    "zhead_1_joint": 500,  # 速度控制模式下刚度为0
    "zhead_2_joint": 500,
}

head_dof_damping = {
    "zhead_1_joint": 50,  # 阻尼系数
    "zhead_2_joint": 50,  # 阻尼系数
}

@dataclass
class DefaultStateCfg:
    # -- limits
    effort_limit: float = 0
    """Maximum effort limit for the actuator. """
    # -- gains
    stiffness: float = 0
    damping: float = 0
    # -- state
    dof_pos: float = 0
    dof_vel: float = 0
    dof_effort: float = 0


KuavoCfg = {
    "arm_names": {
        "zarm_l1_joint",
        "zarm_l2_joint",
        "zarm_l3_joint",
        "zarm_l4_joint",
        "zarm_l5_joint",
        "zarm_l6_joint",
        "zarm_l7_joint",
        "zarm_r1_joint",
        "zarm_r2_joint",
        "zarm_r3_joint",
        "zarm_r4_joint",
        "zarm_r5_joint",
        "zarm_r6_joint",
        "zarm_r7_joint",
    },
    "leg_names": {
        "leg_l1_joint",
        "leg_l2_joint",
        "leg_l3_joint",
        "leg_l4_joint",
        "leg_l5_joint",
        "leg_l6_joint",
        "leg_r1_joint",
        "leg_r2_joint",
        "leg_r3_joint",
        "leg_r4_joint",
        "leg_r5_joint",
        "leg_r6_joint",
    },
    "head_names": {  # 添加头部关节集合
        "zhead_1_joint",
        "zhead_2_joint",
    },
    "default_state": {
        "zarm_l1_joint": DefaultStateCfg(
            effort_limit=1e30,
            stiffness=arm_dof_stiffness["zarm_l1_joint"],
            damping=arm_dof_damping["zarm_l1_joint"],
            dof_pos=arm_initial_pos[0],
        ),
        "zarm_l2_joint": DefaultStateCfg(
            effort_limit=1e30,
            stiffness=arm_dof_stiffness["zarm_l2_joint"],
            damping=arm_dof_damping["zarm_l2_joint"],
            dof_pos=arm_initial_pos[1],
        ),
        "zarm_l3_joint": DefaultStateCfg(
            effort_limit=1e30,
            stiffness=arm_dof_stiffness["zarm_l3_joint"],
            damping=arm_dof_damping["zarm_l3_joint"],
            dof_pos=arm_initial_pos[2],
        ),
        "zarm_l4_joint": DefaultStateCfg(
            effort_limit=1e30,
            stiffness=arm_dof_stiffness["zarm_l4_joint"],
            damping=arm_dof_damping["zarm_l4_joint"],
            dof_pos=arm_initial_pos[3],
        ),
        "zarm_l5_joint": DefaultStateCfg(
            effort_limit=1e30,
            stiffness=arm_dof_stiffness["zarm_l5_joint"],
            damping=arm_dof_damping["zarm_l5_joint"],
            dof_pos=arm_initial_pos[4],
        ),
        "zarm_l6_joint": DefaultStateCfg(
            effort_limit=1e30,
            stiffness=arm_dof_stiffness["zarm_l6_joint"],
            damping=arm_dof_damping["zarm_l6_joint"],
            dof_pos=arm_initial_pos[5],
        ),
        "zarm_l7_joint": DefaultStateCfg(
            effort_limit=1e30,
            stiffness=arm_dof_stiffness["zarm_l7_joint"],
            damping=arm_dof_damping["zarm_l7_joint"],
            dof_pos=arm_initial_pos[6],
        ),
        "zarm_r1_joint": DefaultStateCfg(
            effort_limit=1e30,
            stiffness=arm_dof_stiffness["zarm_r1_joint"],
            damping=arm_dof_damping["zarm_r1_joint"],
            dof_pos=arm_initial_pos[7],
        ),
        "zarm_r2_joint": DefaultStateCfg(
            effort_limit=1e30,
            stiffness=arm_dof_stiffness["zarm_r2_joint"],
            damping=arm_dof_damping["zarm_r2_joint"],
            dof_pos=arm_initial_pos[8],
        ),
        "zarm_r3_joint": DefaultStateCfg(
            effort_limit=1e30,
            stiffness=arm_dof_stiffness["zarm_r3_joint"],
            damping=arm_dof_damping["zarm_r3_joint"],
            dof_pos=arm_initial_pos[9],
        ),
        "zarm_r4_joint": DefaultStateCfg(
            effort_limit=1e30,
            stiffness=arm_dof_stiffness["zarm_r4_joint"],
            damping=arm_dof_damping["zarm_r4_joint"],
            dof_pos=arm_initial_pos[10],
        ),
        "zarm_r5_joint": DefaultStateCfg(
            effort_limit=1e30,
            stiffness=arm_dof_stiffness["zarm_r5_joint"],
            damping=arm_dof_damping["zarm_r5_joint"],
            dof_pos=arm_initial_pos[11],
        ),
        "zarm_r6_joint": DefaultStateCfg(
            effort_limit=1e30,
            stiffness=arm_dof_stiffness["zarm_r6_joint"],
            damping=arm_dof_damping["zarm_r6_joint"],
            dof_pos=arm_initial_pos[12],
        ),
        "zarm_r7_joint": DefaultStateCfg(
            effort_limit=1e30,
            stiffness=arm_dof_stiffness["zarm_r7_joint"],
            damping=arm_dof_damping["zarm_r7_joint"],
            dof_pos=arm_initial_pos[13],
        ),
        "leg_l1_joint": DefaultStateCfg(
            effort_limit=1e30,
            stiffness=leg_dof_stiffness["leg_l1_joint"],
            damping=leg_dof_damping["leg_l1_joint"],
            dof_pos=leg_initial_pos[0],
        ),
        "leg_l2_joint": DefaultStateCfg(
            effort_limit=1e30,
            stiffness=leg_dof_stiffness["leg_l2_joint"],
            damping=leg_dof_damping["leg_l2_joint"],
            dof_pos=leg_initial_pos[1],
        ),
        "leg_l3_joint": DefaultStateCfg(
            effort_limit=1e30,
            stiffness=leg_dof_stiffness["leg_l3_joint"],
            damping=leg_dof_damping["leg_l3_joint"],
            dof_pos=leg_initial_pos[2],
        ),
        "leg_l4_joint": DefaultStateCfg(
            effort_limit=1e30,
            stiffness=leg_dof_stiffness["leg_l4_joint"],
            damping=leg_dof_damping["leg_l4_joint"],
            dof_pos=leg_initial_pos[3],
        ),
        "leg_l5_joint": DefaultStateCfg(
            effort_limit=1e30,
            stiffness=leg_dof_stiffness["leg_l5_joint"],
            damping=leg_dof_damping["leg_l5_joint"],
            dof_pos=leg_initial_pos[4],
        ),
        "leg_l6_joint": DefaultStateCfg(
            effort_limit=1e30,
            stiffness=leg_dof_stiffness["leg_l6_joint"],
            damping=leg_dof_damping["leg_l6_joint"],
            dof_pos=leg_initial_pos[5],
        ),
        "leg_r1_joint": DefaultStateCfg(
            effort_limit=1e30,
            stiffness=leg_dof_stiffness["leg_r1_joint"],
            damping=leg_dof_damping["leg_r1_joint"],
            dof_pos=leg_initial_pos[6],
        ),
        "leg_r2_joint": DefaultStateCfg(
            effort_limit=1e30,
            stiffness=leg_dof_stiffness["leg_r2_joint"],
            damping=leg_dof_damping["leg_r2_joint"],
            dof_pos=leg_initial_pos[7],
        ),
        "leg_r3_joint": DefaultStateCfg(
            effort_limit=1e30,
            stiffness=leg_dof_stiffness["leg_r3_joint"],
            damping=leg_dof_damping["leg_r3_joint"],
            dof_pos=leg_initial_pos[8],
        ),
        "leg_r4_joint": DefaultStateCfg(
            effort_limit=1e30,
            stiffness=leg_dof_stiffness["leg_r4_joint"],
            damping=leg_dof_damping["leg_r4_joint"],
            dof_pos=leg_initial_pos[9],
        ),
        "leg_r5_joint": DefaultStateCfg(
            effort_limit=1e30,
            stiffness=leg_dof_stiffness["leg_r5_joint"],
            damping=leg_dof_damping["leg_r5_joint"],
            dof_pos=leg_initial_pos[10],
        ),
        "leg_r6_joint": DefaultStateCfg(
            effort_limit=1e30,
            stiffness=leg_dof_stiffness["leg_r6_joint"],
            damping=leg_dof_damping["leg_r6_joint"],
            dof_pos=leg_initial_pos[11],
        ),
        "zhead_1_joint": DefaultStateCfg(
            effort_limit=1e30,
            stiffness=head_dof_stiffness["zhead_1_joint"],
            damping=head_dof_damping["zhead_1_joint"],
            dof_pos=0.0,
        ),
        "zhead_2_joint": DefaultStateCfg(
            effort_limit=1e30,
            stiffness=head_dof_stiffness["zhead_2_joint"],
            damping=head_dof_damping["zhead_2_joint"],
            dof_pos=0.0,
        ),
    },
}
