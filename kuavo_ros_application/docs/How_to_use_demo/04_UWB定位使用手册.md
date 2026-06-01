# UWB定位使用手册
## 关于UWB具体的介绍指南，请查阅`ros_localization/nlink_parser`下的README.md

## 在此讲解一下编译方式和启动方式
```bash
# 编译的时候目前的nlink_parser功能包已经放入了机器人的ros_localization定位仓库内
# 直接在kuavo_ros_application当中，执行下述编译即可
catkin build 
```

## 启动方式，请确保uwb连接在了机器人的头部，然后运行
```bash
roslaunch nlink_parser linktrack.launch
```
* 有关linktrack.launch的重要参数
    * port_name 设备串行端口名称，默认值: /dev/ttyUSB0.
    * baud_rate 设备波特率，默认值: 921600

* 如果需要可视化界面，可以打开如下launch
```bash
roslaunch nlink_parser linktrack_rviz.launch
```

## ROS数据流
订阅的话题

* /nlink_linktrack_data_transmission ([std_msgs::String])
你可以通过对该话题发布消息，将数据发送给LinkTrack节点，进而利用数传功能


发布的话题

* /nlink_linktrack_anchorframe0 ([nlink_parser::LinktrackAnchorframe0])


* /nlink_linktrack_tagframe0 ([nlink_parser::LinktrackTagframe0])


* /nlink_linktrack_nodeframe0 ([nlink_parser::LinktrackNodeframe0])


* /nlink_linktrack_nodeframe1 ([nlink_parser::LinktrackNodeframe1])


* /nlink_linktrack_nodeframe2 ([nlink_parser::LinktrackNodeframe2])


* /nlink_linktrack_nodeframe3 ([nlink_parser::LinktrackNodeframe3])


* /nlink_linktrack_nodeframe4 ([nlink_parser::LinktrackNodeframe4])


* /nlink_linktrack_nodeframe5 ([nlink_parser::LinktrackNodeframe5])


* /nlink_linktrack_nodeframe6 ([nlink_parser::LinktrackNodeframe6])

如果收到来自其他节点的数传数据，则 /nlink_linktrack_nodeframe0 话题将会发布消息
其他话题为定位数据话题，当收到协议帧数据，将自动在对应话题上发布消息，协议类型需要在上位机NAssistant上进行配置

## 具体使用案例请参考如下文件，为整体的数据流参考，但不作为最终展示的案例
```bash
/home/kuavo/kuavo_ros_application/src/dynamic_biped/examples/04_use_uwb_to_move
```