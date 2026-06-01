# example案例文件夹讲解
* 在运行案例test文件夹时，请将你想要运行的py文件放入到moveit_interface_plan目录下，如同water_catch_vision_demo.py那样
```bash
├── test_axis_pose_moveit_executor.py  # 输入目标检测结果的末端pose 但是每次规划都是从 当前手臂位置 开始
├── test_axis_pose_moveit.py           # 输入目标检测结果的末端pose 但是每次规划都是从 初始零点位置 开始
├── test_ik_more_point_moveit.py       # 输入多个末端点 然后通过moveit逆解 moveit规划 
├── test_ik_single_point_moveit.py     # 输入单个末端点 然后通过moveit逆解 moveit规划 
├── test_joint_demo_01.py              # 输入单组多个关节角度 moveit规划 
├── test.py                            # 输入多组多个关节角度 moveit规划 
```
* 同时修改open_water_catch.py下的运行文件路径，即可通过下面launch文件启动test案例
```bash
roslaunch moveit_interface_plan vision_catch_water_demo.launch
```