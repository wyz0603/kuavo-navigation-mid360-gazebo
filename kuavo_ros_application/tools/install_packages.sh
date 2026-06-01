#!/bin/bash
## apt自动更新相应的库
## 使用方法如下：
## chmod +x install_packages.sh
## ./install_packages.sh

sudo apt update
sudo apt-get install python-catkin-tools

sudo apt-get install python3-pip
pip config set global.index-url https://pypi.tuna.tsinghua.edu.cn/simple
