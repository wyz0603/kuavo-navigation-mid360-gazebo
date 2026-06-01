## 简易说明文档
- 2025-06-06
### 检查设备识别情况

```bash
arecord -l
```
### 查看声卡参数

```bash
arecord -D hw:0,0 --dump-hw-params
```
### ros录音测试
```bash
cd ~/kuavo_ros_application/src/ros_audio/kuavo_audio_recorder/scripts
chmod +x test_record.sh
./test_record.sh
```
录制好的文件在`~/kuavo_ros_application/src/ros_audio/kuavo_audio_recorder/scripts`目录下。

### 声卡被占用：

```bash
sudo alsa force-reload

sudo pkill -f pulseaudio
```

