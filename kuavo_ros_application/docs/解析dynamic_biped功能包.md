# dynamic_biped功能包完成过程如下
## 快速构建 & 快速启动
### (1) glone & build your workspace
```bash
git clone https://www.lejuhub.com/ros-application-team/kuavo_ros_application.git   # Clone Apriltag library
cd ~/kuavo_ros_application
git checkout dev     # 切换为dev分支
source /opt/ros/noetic/setup.bash
catkin_make_isolated       # Build all packages in the workspace (catkin_make_isolated will work also)
```
### (2) Launch启动(一键启动launch所有功能)
```bash
cd ~/kuavo_ros_application
source /opt/ros/noetic/setup.bash
source ~/kuavo_ros_application/devel_isolated/setup.bash 

roslaunch dynamic_biped sensor_robot_enable.launch 
```
## (3) 编写launch文件如下（解析如下）
* sensor_robot_enable.launch
* 请注意，关于static_transform_publisher类型的静态tf转换不能改动，涉及到相机坐标系合并到机器人坐标的tf树
```xml
<!-- sensor_robot_enable.launch -->
<launch>
    <!-- Launch realsense2_camera for head camera -->
    <include file="$(find realsense2_camera)/launch/rs_camera.launch" >
        <arg name="color_width"   value="640"/>
        <arg name="color_height"  value="480"/>
        <arg name="color_fps"     value="30"/>
        <arg name="depth_width"   value="848"/>
        <arg name="depth_height"  value="480"/>
        <arg name="depth_fps"     value="30"/>
        <arg name="enable_infra"        default="false"/>
        <arg name="enable_infra1"       default="false"/>
        <arg name="enable_infra2"       default="false"/>
        <arg name="enable_sync"   value="true"/>
        <arg name="align_depth"   value="true"/>
        <arg name="enable_pointcloud"   value="false"/>
    </include>

    <!-- tf2_ros 静态转换 发布urdf里面的head_camera 和 相机坐标系下的camera_link 进行对齐 -->
    <node pkg="tf2_ros" type="static_transform_publisher" name="camera_to_real_frame" args="0 0 0 0 0 0 head_camera camera_link" />

    <!-- 机器人全身关节tf树 biped_s4.urdf 发布 -->
    <include file="$(find urdf_tutorial)/launch/display.launch">
        <arg name="model" value="$(find biped_s4)/urdf/biped_s4.urdf" />
    </include>

    <!-- 启动yolov5 目标检测 及 分割节点 -->
    <node pkg="kuavo_vision_object" type="realsense_yolo_segment_ros.py" name="realsense_yolo_segment_node" output="screen">
    </node>

    <!-- 启动yolov5 目标转换功能 -->
    <node pkg="kuavo_vision_object" type="realsense_yolo_transform_torso.py" name="realsense_yolo_transform_torso_node" output="screen">
    </node>

    <!-- 启动yolov5 点云分割节点 -->
    <node pkg="kuavo_yolo_point2d" type="point_cloud_mask_node.py" name="point_cloud_mask_node" output="screen">
    </node>

    <!-- 启动 apriltag_ros continuous_detection -->
    <include file="$(find apriltag_ros)/launch/continuous_detection.launch">
      <arg name="camera_name" value="/camera/color" />
      <arg name="image_topic" value="image_raw" />
    </include>

    <!-- 启动 ARControlNode -->
    <node pkg="ar_control" type="ar_control_node.py" name="ar_control_node" output="screen" respawn="true" respawn_delay="5" />

    <!-- 启动 播放音乐服务 -->
    <node pkg="kuavo_audio_player" type="loundspeaker.py" name="play_music_node" output="screen" respawn="true" respawn_delay="5"/>
</launch>
```