# 准备工作

* 安装 python 依赖库

```bash
pip3 install -r requirements.txt
```

* 开启摄像头

默认使用机器人上面的 realsense 摄像头，可以使用以下命令启动摄像头 ros 程序

```bash
roslaunch realsense2_camera rs_camera.launch
```
然后通过 `/camera/color/image_raw` 可以获取到摄像头信息

# 人脸匹配识别方案

将需要识别的人脸放到`faces/`下面，文件名命名为人名，图片格式指定为png。

按照下面命令操作即可：

```bash
python3 -m pip uninstall matplotlib
python3 -m pip install matplotlib==3.5.1
wget https://kuavo.lejurobot.com/kuavo_research_editiion/face_models/buffalo_l.zip
mkdir -p ~/.insightface/models/buffalo_l
unzip buffalo_l.zip -d ~/.insightface/models/buffalo_l
python3 asr_re_tts.py
``
程序会先进行语音检测，当识别到语音中包含“是谁”两个字时，开始人脸检测
代码中会提取人的面部特征向量与模板匹配，如有匹配的人脸，会有语音播报：
“xxx, 你好”


