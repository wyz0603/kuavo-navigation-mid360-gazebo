# 模型部署
#### Step1：安装opencv依赖库及EdgeBoard DK-1A推理工具PPNC(如已安装，可跳过此步)
* 打开终端，执行以下命令安装PPNC。
```bash
sudo apt update
sudo apt install libopencv-dev -y
sudo apt install python3-opencv -y
sudo apt install ppnc-runtime -y
```
#### Step2：安装PaddlePaddle(如已安装，可跳过此步)
* 打开终端，执行以下命令安装PaddlePaddle。
```bash
mkdir Downloads
cd Downloads
wget https://bj.bcebos.com/pp-packages/whl/paddlepaddle-2.4.2-cp38-cp38-linux_aarch64.whl  
sudo pip install paddlepaddle-2.4.2-cp38-cp38-linux_aarch64.whl -i https://pypi.tuna.tsinghua.edu.cn/simple
```

#### Step3：下载python部署源码包。(源码已包含此部分，跳过此步)
https://bj.bcebos.com/ppdeploy/ppdeploy1.1/SDK/PPDeploy11_yolov3_PaddleDetection2.6_Paddle2.4.1_Ver1.0.0_python.zip

#### Step4：安装依赖库。
* 在终端输入以下命令，进入yolov3-python目录，并安装依赖库：
```bash
cd yolov3-python
sudo pip install -r requirements.txt -i https://pypi.tuna.tsinghua.edu.cn/simple
```

* 说明:
参数`-i https://pypi.tuna.tsinghua.edu.cn/simple`表示此次使用清华源进行安装。由于网络原因，直接使用默认源安装可能会出现报错。

* 同时yolov3的部署额外需要onnxruntime软件，安装：
```bash
sudo apt update
sudo apt install onnxruntime
```
#### Step5：配置config.json文件（无更改可略过）
* 将模型生产阶段产生的model.nb、model.json、model.po、model.onnx模型文件传输至板卡，置于`yolov3-python/model`文件夹
* model目录下修改config.json配置
```json
{
    "mode": "professional",
    "model_dir": "./model", 
    "model_file": "model"
}
```
* 参数说明:
```
    - mode: 固定为"professional"
    - model_dir：传输至板卡的模型文件(model.json、model.nb、model.onnx、model.po)的目录
    - model_file: 传输至板卡的四个模型文件的文件名，固定为model
```

#### Step6：运行推理代码。
* 确保当前位于yolov3-python目录下：
```shell
    sudo python3 tools/infer_demo.py \
    --config ./model/config.json \
    --infer_yml ./model/infer_cfg.yml \
    --test_image ./test_images/000000025560.jpg \
    --visualize \
    --with_profile
```

* 命令行选项参数如下：
```
    - config: 上文建立的config.json的路径
    --infer_yml: 模型导出时生成的infer_cfg.yml文件
    - test_image: 测试图片路径
    - visualize: 是否可视化，若设置则会在该路径下生成vis.jpg渲染结果，默认不生成
    - with_profile: 是否推理耗时，若设置会输出包含前处理、模型推理和后处理的总耗时，默认不输出
```

#### Step7：查看推理结果。 
* 在`yolov3-python`目录下可以看到新增一个名为vis.jpg的推理结果文件。