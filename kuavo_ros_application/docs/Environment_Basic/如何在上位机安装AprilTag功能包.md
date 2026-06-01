# 安装过程如下：
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
git clone https://github.com/AprilRobotics/apriltag.git       # Clone Apriltag library
git clone https://github.com/AprilRobotics/apriltag_ros.git   # Clone Apriltag ROS wrapper

# （二）开始编译，因为原始apriltag是plain（CMake）依赖于CMake的，所以在ros1当中使用catkin_make_isolated命令进行多工作空间编译
cd ~/catkin_ws 
rosdep install --from-paths src --ignore-src -r -y  # Install any missing packages
catkin_make_isolated # 非均质工作空间编译（多工作空间，每个功能包都是一个独立的工作空间）

# （三）启动节点(后面两个参数为RealSense深度相机发布的topic名字，分为是相机的Image类型图像和Camera_info相机信息)，检查是否存在报错
roslaunch apriltag_ros continuous_detection.launch camera_name:=/camera/color image_topic:=image_raw
```

# 对于要检测tag id来说，你需要重点关注apriltag_ros功能包下的两个文件
#### tags.yaml
* 关注参数tag_family，这是标签属于哪个家族（类别）
* 其他的参数可以暂时先不管，作为默认值就好
```yaml
# AprilTag 3 code parameters
# Find descriptions in apriltag/include/apriltag.h:struct apriltag_detector
#                      apriltag/include/apriltag.h:struct apriltag_family
tag_family:        'tag36h11' # options: tagStandard52h13, tagStandard41h12, tag36h11, tag25h9, tag16h5, tagCustom48h12, tagCircle21h7, tagCircle49h12
tag_threads:       2          # default: 2
tag_decimate:      1.0        # default: 1.0
tag_blur:          0.0        # default: 0.0
tag_refine_edges:  1          # default: 1
tag_debug:         0          # default: 0
max_hamming_dist:  2          # default: 2 (Tunable parameter with 2 being a good choice - values >=3 consume large amounts of memory. Choose the largest value possible.)
# Other parameters
publish_tf:        true       # default: false
transport_hint:    "raw"      # default: raw, see http://wiki.ros.org/image_transport#Known_Transport_Packages for options
```
---
#### settings.yaml
* 对于刚下载的功能包，下面的这个参数默认为空，不检测任何tag，所以我们要进行修改
```yaml
standalone_tags:

  [

  ]

```
* 把原来的config/tags.yaml的standalone_tags参数改为
```yaml
standalone_tags:
  [
    {id: 0, size: 0.046, name: 'tag_0'},
    {id: 1, size: 0.046, name: 'tag_1'},
    {id: 2, size: 0.046, name: 'tag_2'},
    {id: 3, size: 0.046, name: 'tag_3'},
    {id: 4, size: 0.046, name: 'tag_4'},
    {id: 5, size: 0.046, name: 'tag_5'},
    {id: 6, size: 0.046, name: 'tag_6'},
    {id: 7, size: 0.046, name: 'tag_7'},
    {id: 8, size: 0.046, name: 'tag_8'},
    {id: 9, size: 0.046, name: 'tag_9'},
  ]
```
* id为具体要检测的标签id，
* size和实际检测的标签的尺寸有关（size十分重要，对于识别来说影响到了tag码在坐标系的是否为正确的位置，所以请务必与实际打印识别标签的尺寸用卷尺进行测量和修改）
* name 就是识别过后发布通过tf广播器发布具体从tag码到相机坐标系转换的tf名称