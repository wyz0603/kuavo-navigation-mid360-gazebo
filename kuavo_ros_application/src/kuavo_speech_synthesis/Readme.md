# 语音合成
## X86 架构下语音合成
该项目主要在 X86 平台上以实现文本为输入，音频为输出的功能为目的，调用本地模型来离线合成音频。
## 环境依赖
- 该节点需要使用离线模型 "speech_sambert-hifigan_tts_zhida_zh-cn_16k" 进行推理合成，故而需要依赖相关环境。
- 模型部署过程较为繁琐，本项目提供一个安装脚本进行部署：[install](./install.sh)
### 部署说明
- 进入目标目录，并切换为 root 用户，然后执行脚本
```bash
    cd /home/kuavo/kuavo_ros_application/src/kuavo_speech_synthesis
    sudo su && ./install.sh
```
- 由于离线模型较大，下载过程可能需要 10～20 分钟，请耐心等待，安装部署完成后会有绿色文字的成功提醒。

## ARM 架构下语音合成
该项目主要在 ARM 平台上以实现文本为输入，音频为输出的功能为目的，调用本地模型来离线合成音频。
## 环境依赖
- 该节点需要使用离线模型 "sherpa_onnx/matcha-icefall-zh-baker" 进行推理合成，故而需要依赖相关环境。
- 模型文件已上传至云端，第一次运行时请确保网络连接通畅，以保证模型的拉取和解压。
### 部署说明
- 执行下列命令，安装相关功能包
```bash
    pip install --upgrade soundfile librosa scipy
    pip install sherpa-onnx
```
## 使用说明
### 编译
```bash
    cd /home/kuavo/kuavo_ros_application
    catkin build kuavo_speech_synthesis
```
### 运行
#### 于 X86 架构下的环境运行：
- 直接使用 rosrun 启动：
```bash
    source /home/kuavo/kuavo_ros_application/devel/setup.bash
    rosrun kuavo_speech_synthesis kuavo_speech_synthesis_node.py
```
- 或者使用 launch 命令一键运行，并提供相关参数：
```bash
    source /home/kuavo/kuavo_ros_application/devel/setup.bash
    roslaunch kuavo_speech_synthesis speech_synthesis.launch
```
#### 于 ARM 架构下的环境运行：
- 直接使用 rosrun 启动：
```bash
    source /home/kuavo/kuavo_ros_application/devel/setup.bash
    rosrun kuavo_speech_synthesis kuavo_speech_synthesis_node_arm.py
```
- 或者使用 launch 命令一键运行，并提供相关参数：
```bash
    source /home/kuavo/kuavo_ros_application/devel/setup.bash
    roslaunch kuavo_speech_synthesis speech_synthesis_arm.launch
```
### 使用
- 目前采用服务的形式接收 string 文本参数，并进行推理合成，故而消息结构如下：
```bash
# 请求部分
string data
---
# 响应部分
bool success   
```
### 调用
- 服务名为：“speech_synthesis”
- 命令行调用：
```bash
    rosservice call /speech_synthesis "data: '你好'"
```
### 结果
- 本项目会自动合成音频，可以搭配 kuavo_audio_player 项目进行语音的播放。
