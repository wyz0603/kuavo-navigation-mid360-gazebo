# kuavo_humanoid_websocket_sdk_server 服务自启部署脚本说明

外部 PC 使用 kuavo_humanoid_websocket_sdk 通过 websocket 方式访问机器人内部的话题和服务，在使用 SDK 时需要在机器人上先启动 rosbridge_server 服务器，rosbridge_server 服务器部署在上位机。

## 临时使用

1. `cd <kuavo_ros_application>` 到 kuavo_ros_application 工程目录
2. ``export KUAVO_ROS_APPLICATION_WS_PATH= `pwd` `` 设置环境变量
3. `./tools/deploy_tools/humanoid_websocket_sdk_server/start_websocket_server.sh` 开启 rosbridge_server 服务器

## 开机自启部署

1. `cd <kuavo_ros_application>` 到 kuavo_ros_application 工程目录
2. `./tools/deploy_tools/humanoid_websocket_sdk_server/deploy_autostart_websocket_server.sh`
