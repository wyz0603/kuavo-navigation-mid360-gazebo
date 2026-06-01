if [ -f "CATKIN_IGNORE" ]; then
    echo "检测到CATKIN_IGNORE文件，正在删除..."
    rm CATKIN_IGNORE
else
    arch=$(uname -m)
    if [ "$arch" = "x86_64" ]; then
        echo "检测到系统架构为amd64，正在运行install.sh..."
        bash install.sh
    elif [[ "$arch" == arm* || "$arch" == aarch64 ]]; then
        echo "检测到系统架构为ARM，正在安装依赖包..."
        pip install --upgrade soundfile librosa scipy sherpa-onnx
        if [ $? -eq 0 ]; then
            echo "依赖包安装成功。"
        else
            echo "依赖包安装失败，请检查网络连接或pip配置。"
            exit 1
        fi
    else
        echo "未识别的系统架构: $arch"
        exit 1
    fi
fi

echo "开始编译 kuavo_speech_synthesis 包..."
catkin build kuavo_speech_synthesis
if [ $? -eq 0 ]; then
    echo -e "\033[32mkuavo_speech_synthesis 编译成功！\033[0m"
else
    echo -e "\033[31mkuavo_speech_synthesis 编译失败，请检查错误信息。\033[0m"
    exit 1
fi

