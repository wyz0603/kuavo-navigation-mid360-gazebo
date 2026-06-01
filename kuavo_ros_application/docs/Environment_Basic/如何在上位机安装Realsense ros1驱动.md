## 因为官网  [InterReal Sense 仓库](https://github.com/IntelRealSense/realsense-ros/tree/ros1-legacy)  明确表示后面主推ROS2, 对于ROS1版本不再进行维护
*所以对于该版本需要下载源码进行编译*
`选择ros1-legcy分支`

```bash
# 建立workspace（此步骤可跳过）
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src/
catkin_init_workspace 
cd ..
catkin_make
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc

# （一） 在catkin_ws/src/下载源程序
cd src
git clone https://github.com/IntelRealSense/realsense-ros.git -b ros1-legacy # 记得切换分支为ros1-legacy realsense库适配ROS1的版本
git clone https://github.com/pal-robotics/ddynamic_reconfigure.git           # 允许动态更改ROS节点参数的tool

# （二） catkin_make_isolated 编译 因为AprilTag的原始CMake库是非均质工作空间
cd ~/catkin_ws && catkin_make_isolated

# （三） source ros1工作空间下的功能包 引入到当前目录
source ~/ros_catkin_ws/install_isolated/local_setup.bash

# （四） 测试launch 
roslaunch realsense2_camera demo_pointcloud.launch # 测试点云数据
roslaunch realsense2_camera rs_camera.launch   # **启动所有相机功能**
roslaunch realsense2_camera rs_rgbd.launch  #  **仅启动彩色和深度相机**
roslaunch realsense2_camera rs_color.launch # **启动只有彩色相机**
 

# （五） 在打开的rviz里面查看其他图像数据是否正常

# 其他测试
## 查看发布的topic
rostopic list

## 查看相机内参
# 方法一
rostopic echo /camera/color/camera_info 
rostopic echo /camera/aligned_depth_to_color/camera_info
```

## 二、针对双摄像头的启动，可以查看kuavo_robot_ros/launch/double_camera_test.launch文件
```xml
<!-- double_camera_test.launch -->
<launch>
    <!-- Launch realsense2_camera for head camera -->
    <include file="$(find realsense2_camera)/launch/rs_camera.launch" >
        <arg name="camera" value="/head/camera" />
        <arg name="serial_no" value="239722074424" /> 
    </include>

    <!-- Launch realsense2_camera for chest camera -->
    <include file="$(find realsense2_camera)/launch/rs_camera.launch" >
        <arg name="camera" value="/chest/camera" />
        <arg name="serial_no" value="342522070992" />
    </include>
</launch>
```
* 参数 name      是你指定给这个相机的名字
* 参数 serial_no 是相机实际连接至nuc后，通过命令行查看的相机串口序列号
```bash
serial_no 参数用于指定要连接的相机的序列号。在你提供的设备信息中，每个相机都有一个唯一的序列号。在这种情况下，你可以将相应相机的序列号放入 serial_no 参数中。
可以通过终端输入 rs-enumerate-devices 查看当前连接nuc的realsense设备的信息，其中的Serial Number 就是相机的唯一指定序列号
```
* 举例：
```bash
kuavo@kuavo-NUC12WSKi7:~/kuavo_ros_application$ rs-enumerate-devices 
Device info: 
    Name                          :     Intel RealSense D435
    Serial Number                 :     342522070992
    Firmware Version              :     5.13.0.55
    Recommended Firmware Version  :     5.15.0.2
    Physical Port                 :     /sys/devices/pci0000:00/0000:00:14.0/usb4/4-1/4-1:1.0/video4linux/video6
    Debug Op Code                 :     15
    Advanced Mode                 :     YES
    Product Id                    :     0B07
    Camera Locked                 :     YES
    Usb Type Descriptor           :     3.2
    Product Line                  :     D400
    Asic Serial Number            :     345423023264
    Firmware Update Id            :     345423023264

Stream Profiles supported by Stereo Module
 Supported modes:
    stream       resolution      fps       format   
    Infrared 1    1280x800      @ 30Hz     Y8
    Infrared 1    1280x800      @ 25Hz     Y16
    Infrared 1    1280x800      @ 15Hz     Y16
    Infrared 1    1280x800      @ 15Hz     Y8
    Infrared 1    1280x720      @ 30Hz     Y8
    Infrared 1    1280x720      @ 15Hz     Y8
    Infrared 1    1280x720      @ 6Hz      Y8
    Infrared 1    848x480       @ 90Hz     Y8
    Infrared 1    848x480       @ 60Hz     Y8
。。。
。。。(省略)
```