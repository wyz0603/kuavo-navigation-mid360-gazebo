# Dockerfile 用于关节校准环境

# 使用ros noetic基础镜像作为基础镜像
FROM ros:noetic-ros-base

# 设置非交互式安装
ENV DEBIAN_FRONTEND=noninteractive

# 安装必要的工具
RUN apt-get update && apt-get install -y \
    wget \
    && rm -rf /var/lib/apt/lists/*

# 安装miniconda3
RUN mkdir -p /opt/miniconda3 \
    && wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh -O /opt/miniconda3/miniconda.sh \
    && bash /opt/miniconda3/miniconda.sh -b -u -p /opt/miniconda3 \
    && rm /opt/miniconda3/miniconda.sh

# 设置环境变量
ENV PATH="/opt/miniconda3/bin:${PATH}"

# 初始化conda
RUN conda init bash

# 创建工作目录
WORKDIR /app

# 复制需要的文件
COPY requirements.txt /app/
COPY *.py /app/
COPY bags /app/bags

# 安装PyQt5所需的依赖
RUN apt-get update && apt-get install -y \
    libgl1-mesa-glx \
    libxrender1 \
    libxext6 \
    libx11-xcb1 \
    libxkbcommon-x11-0 \
    libdbus-1-3 \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# 安装apriltag ros消息
RUN apt-get update && apt-get install -y \
    ros-noetic-apriltag-ros \
    && rm -rf /var/lib/apt/lists/*

# 创建conda环境并安装依赖项
RUN conda create -n joint_cali python=3.8 -y \
    && . /opt/miniconda3/etc/profile.d/conda.sh \
    && conda activate joint_cali \
    && conda install -c conda-forge --file requirements.txt -y

# 设置容器启动命令
SHELL ["/bin/bash", "-c"]
ENTRYPOINT ["/bin/bash", "-c", "source /opt/miniconda3/etc/profile.d/conda.sh && conda activate joint_cali && exec \"$@\"", "-"]
CMD ["/app/arm_cali.py"]
