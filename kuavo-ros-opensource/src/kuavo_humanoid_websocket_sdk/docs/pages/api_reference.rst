.. _api-reference:

.. warning::

    Before running any code examples, make sure to start the robot first by executing either:
    
    - For simulation: ``roslaunch humanoid_controllers load_kuavo_mujoco_sim.launch`` (Example command)
    - For real robot: ``roslaunch humanoid_controllers load_kuavo_real.launch`` (Example command)

.. note::
    If using websocket mode, you need to start the rosbridge server on robot first: ``roslaunch rosbridge_server rosbridge_websocket.launch`` (Example command)


*************
API Reference
*************
.. currentmodule:: kuavo_humanoid_sdk

.. autoclass:: KuavoSDK
    :members:
    :undoc-members:
    :show-inheritance:

.. autoclass:: KuavoRobot
    :members:
    :undoc-members:
    :show-inheritance:

.. autoclass:: KuavoRobotInfo
    :members:
    :undoc-members:
    :show-inheritance:

.. autoclass:: KuavoRobotState
    :members:
    :undoc-members:
    :show-inheritance:

.. autoclass:: KuavoRobotArm
    :members:
    :undoc-members:
    :show-inheritance:

.. autoclass:: KuavoRobotHead
    :members:
    :undoc-members:
    :show-inheritance:

.. autoclass:: KuavoRobotVision
    :members:
    :undoc-members:
    :show-inheritance:

.. autoclass:: KuavoRobotAudio
    :members:
    :undoc-members:
    :show-inheritance:

.. autoclass:: KuavoRobotTools
    :members:
    :undoc-members:
    :show-inheritance:

.. autoclass:: DexterousHand
    :members:
    :undoc-members:
    :show-inheritance:

.. autoclass:: TouchDexterousHand
    :members:
    :undoc-members:
    :show-inheritance:

.. autoclass:: LejuClaw
    :members:
    :undoc-members:
    :show-inheritance:

.. autoclass:: KuavoRobotObservation
    :members:
    :undoc-members:
    :show-inheritance:

.. automodule:: kuavo_humanoid_sdk.interfaces.data_types
    :members:

