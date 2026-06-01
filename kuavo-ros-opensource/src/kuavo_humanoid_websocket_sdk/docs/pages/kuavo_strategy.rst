.. _kuavo-strategy:

.. warning::

    Before running any code examples, make sure to start the robot first by executing either:
    
    - For simulation: ``roslaunch humanoid_controllers load_kuavo_mujoco_sim.launch`` (Example command)
    - For real robot: ``roslaunch humanoid_controllers load_kuavo_real.launch`` (Example command)

.. note::
    If using websocket mode, you need to start the rosbridge server on robot first: ``roslaunch rosbridge_server rosbridge_websocket.launch`` (Example command)

************************
Kuavo Strategy 策略模块
************************

基础策略接口
====================
.. currentmodule:: kuavo_humanoid_sdk.kuavo_strategy

.. autoclass:: kuavo_humanoid_sdk.kuavo_strategy.kuavo_strategy.KuavoRobotStrategyBase
    :members:
    :undoc-members:
    :show-inheritance:

箱子抓取策略
====================
.. currentmodule:: kuavo_humanoid_sdk.kuavo_strategy.grasp_box

箱子信息数据结构
------------------------
.. autoclass:: kuavo_humanoid_sdk.kuavo_strategy.grasp_box.grasp_box_strategy.BoxInfo
    :members:
    :undoc-members:
    :show-inheritance:
    :noindex:

箱子抓取策略类
------------------------
.. autoclass:: kuavo_humanoid_sdk.kuavo_strategy.grasp_box.grasp_box_strategy.KuavoGraspBox
    :members:
    :undoc-members:
    :show-inheritance:

使用示例
============

以下是一个使用箱子抓取策略的基本示例:

.. code-block:: python

    import time
    from kuavo_humanoid_sdk import KuavoRobot, KuavoRobotState, KuavoRobotTools, KuavoRobotVision
    from kuavo_humanoid_sdk.interfaces.data_types import KuavoPose, AprilTagData, PoseQuaternion
    from kuavo_humanoid_sdk.kuavo_strategy.grasp_box.grasp_box_strategy import KuavoGraspBox, BoxInfo

    # 初始化机器人及相关组件
    robot = KuavoRobot()
    robot_state = KuavoRobotState()
    robot_tools = KuavoRobotTools()
    robot_vision = KuavoRobotVision()
    
    # 初始化箱子抓取策略
    grasp_strategy = KuavoGraspBox(robot, robot_state, robot_tools, robot_vision)
    
    # 创建AprilTag数据对象
    target_april_tag = AprilTagData(
        id=[42],  # AprilTag ID
        size=[0.1],  # AprilTag 标签尺寸
        pose=[PoseQuaternion(
            position=(0.5, 0.0, 0.4),  # 位置
            orientation=(0.0, 0.0, 0.0, 1.0)  # 四元数方向
        )]
    )
    
    # 使用头部寻找目标
    find_success = grasp_strategy.head_find_target(
        target_april_tag, 
        max_search_time=15.0,
        search_pattern="rotate_head"
    )
    
    if find_success:
        print("目标找到成功!")