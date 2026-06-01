FROM nvidia/opengl:1.2-glvnd-runtime-ubuntu20.04

ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Asia/Shanghai

# GPU support
ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES graphics,utility,compute
###################  first system wide configuration ###################################
# change timezone (this is very important otherwise many ROS topic time will be strange)
# TODO: please modify it to your timezone accordingly
# ENV TZ=Asia/Shanghai
# ENV TZ=America/New_York
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

RUN apt-get update -y && apt-get install ca-certificates -y && update-ca-certificates && \
    sh -c 'echo "deb https://mirrors.ustc.edu.cn/ubuntu/ focal main restricted universe multiverse\ndeb https://mirrors.ustc.edu.cn/ubuntu/ focal-updates main restricted universe multiverse\ndeb https://mirrors.ustc.edu.cn/ubuntu/ focal-backports main restricted universe multiverse\ndeb https://mirrors.ustc.edu.cn/ubuntu/ focal-security main restricted universe multiverse" > /etc/apt/sources.list' && \
    apt-get update -y && DEBIAN_FRONTEND=noninteractive apt-get install -y tzdata && \
    apt-get install -y dirmngr gnupg2 && \
    sh -c 'echo "deb http://mirrors.ustc.edu.cn/ros/ubuntu/ focal main" > /etc/apt/sources.list.d/ros-latest.list' && \
    apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 &&\
    rm -rf /var/lib/apt/lists/*

# >>> Tools <<<
ENV ZSH_CUSTOM=/root/.oh-my-zsh/custom
# zsh
RUN apt-get update -y \
    && apt-get install -y git wget \
    && git clone https://github.com/ohmyzsh/ohmyzsh.git /root/.oh-my-zsh \
    && git clone https://github.com/zsh-users/zsh-autosuggestions.git $ZSH_CUSTOM/plugins/zsh-autosuggestions \
    && git clone https://github.com/zsh-users/zsh-syntax-highlighting.git $ZSH_CUSTOM/plugins/zsh-syntax-highlighting \
    && sh -c "$(wget -O- https://github.com/deluan/zsh-in-docker/releases/download/v1.1.5/zsh-in-docker.sh)" -- \
    -t ys \
    -p git \
    -p zsh-syntax-highlighting \
    -p zsh-autosuggestions

# Gnome terminal support
RUN apt-get update -y \
    && apt-get install -y gnome-terminal \
    dbus-x11 \
    libcanberra-gtk-module \ 
    libcanberra-gtk3-module

# ros
RUN apt-get update -y \
    && apt-get install -y wget python3 python3-yaml python3-distro\
    && apt-get install -y ros-noetic-ros-base python3-rosdep python3-rosinstall \
    python3-rosinstall-generator python3-wstool python3-catkin-tools build-essential \
    && apt-get install ros-noetic-plotjuggler* -y \
    && apt-get install -y ros-noetic-eigen-conversions \
    && apt-get install -y ros-noetic-roslint \
    && apt-get install -y ros-noetic-costmap-converter \
    && apt-get install -y ros-noetic-move-base-msgs \
    && apt-get install -y ros-noetic-mbf-costmap-core \
    && apt-get install -y ros-noetic-mbf-msgs \
    && apt-get install -y ros-noetic-teb-local-planner \
    && apt-get install -y ros-noetic-interactive-markers ros-noetic-tf ros-noetic-tf2-ros ros-noetic-urdf \
    ros-noetic-kdl-parser ros-noetic-robot-state-publisher  ros-noetic-rqt-graph \
    && apt-get install -y ros-noetic-realtime-tools ros-noetic-ros-control ros-noetic-xacro\
    && echo 'source /opt/ros/noetic/setup.bash' >> /root/.bashrc \
    && echo 'source /opt/ros/noetic/setup.zsh' >> /root/.zshrc \
    && echo "chooses:" > fish_install.yaml \
    && echo "- {choose: 3, desc:''}" >> fish_install.yaml 

# 注释掉 . ~/.zshrc 2>/dev/null    
RUN wget http://fishros.com/install -O fishros \ 
    && sed -i '/\. ~\/\.zshrc 2>\/dev\/null/s/^/#/' fishros \
    && /bin/bash fishros

RUN rm -rf fish_install.yaml \
    && rm -rf /var/lib/apt/lists/*  /tmp/* /var/tmp/* \
    && apt clean && apt autoclean 

ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

# KUAVO-SLAM.move_base
RUN apt-get update -y \
    && apt-get install -y ros-noetic-ros-babel-fish ros-noetic-stage-ros libnetpbm10-dev \
    && apt-get install -y libbullet-dev libsdl1.2-dev libsdl-image1.2-dev \
    && apt-get install -y python3-sqlalchemy libasio-dev libwebsocketpp-dev nlohmann-json3-dev \
    && apt-get install -y ros-noetic-control-box-rst

# 安装 build_kuavo_slam_ws.sh 脚本中涉及的依赖
RUN apt-get update -y \
    && apt-get install -y \
        cmake \
        git \
        octovis \
        ros-noetic-tf2-sensor-msgs \
        ros-noetic-joy \
        ros-noetic-plotjuggler \
        ros-noetic-interactive-markers \
        ros-noetic-tf \
        ros-noetic-tf2-ros \
        ros-noetic-urdf \
        ros-noetic-kdl-parser \
        ros-noetic-robot-state-publisher \
        ros-noetic-rqt-graph \
        ros-noetic-realtime-tools \
        ros-noetic-ros-control \
        ros-noetic-xacro \
        ros-noetic-octomap-mapping \
        ros-noetic-octomap-rviz-plugins \
        ros-noetic-pointcloud-to-laserscan \
    && rm -rf /var/lib/apt/lists/*

# 安装 Python3 及相关依赖（包含build_kuavo_slam_ws.sh中的依赖）
RUN mkdir -p /root/.pip && \
    echo "[global]" > /root/.pip/pip.conf && \
    echo "index-url = https://mirrors.aliyun.com/pypi/simple/" >> /root/.pip/pip.conf && \
    echo "trusted-host = mirrors.aliyun.com" >> /root/.pip/pip.conf

RUN apt-get update -y \
    && apt-get install -y python3-yaml python3-distro python3-tk \
    && pip3 install --upgrade pip \
    && pip3 install --trusted-host mirrors.aliyun.com  cryptography pyOpenSSL certifi numpy==1.24.4 soundfile librosa scipy sherpa-onnx open3d==0.13.0 \
    && rm -rf /var/lib/apt/lists/*

RUN echo 'export ROBOT_VERSION=45' >> /root/.bashrc \
    && echo 'export ROBOT_VERSION=45' >> /root/.zshrc \
    && echo 'export DISABLE_ROS1_EOL_WARNINGS=ON' >> /root/.bashrc \
    && echo 'export DISABLE_ROS1_EOL_WARNINGS=ON' >> /root/.zshrc

# 初始化 rosdep
# RUN rosdep init

# Copy TLS certificates for Foxglove
COPY .tls /root/.tls
RUN cp /root/.tls/rootCA.pem /usr/local/share/ca-certificates/rootCA.crt 

# set default shell to zsh
RUN chsh -s $(which zsh) 
WORKDIR /root/kuavo_ws

