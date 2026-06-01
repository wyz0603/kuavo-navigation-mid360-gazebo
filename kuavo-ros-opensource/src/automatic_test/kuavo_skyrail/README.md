# Kuavo_SkyRail  
# （一）获取动捕数据  
仓库地址：https://www.lejuhub.com/zhanglongbo/motioncapture.git  
1. 使用动捕windows上位机，连接镜头，启动播放 ，确认3D视图里面 能看到 car 和 robot 两个刚体，看不到也不要紧，可以删除右边资产库，冻结帧重新添加刚体模型。一共需要两个模型（名称一定为 car 和robot），天轨吊车（当前是依照链束袋建模）和机器人（有工装），建模成功后 勾选右下角sdk播放 ，在设置里面找到广播的ip  
2. 在另一台ubuntu上位机，运行上述仓库，确保两台电脑在一个ip网段，按提示输入上述Ip。既可启动。  
3. 仓库启动后，可通过plotjuggler，来确认是否正确获得了 car 和robot 的六维坐标

# (二)启动天轨系统 （前提是动捕系统已经顺利启动，本仓库仅对数据做订阅，动捕发送数据在另一个仓库）
工程实现了，通过使用动捕获取到的xy轴坐标，控制吊车进行二维跟踪机器人的功能 同时包含键盘控制
## Kuavo about Automated test of the sky rail


`nimservos_controller`电机驱动功能包
* 包含电机canopen python 驱动的SDK
* 包含电机 ros调用的控制接口


`user_pkg` 用户功能包
* 用户键盘监听 用于发布控制命令


## 使用说明:

1. **开启新终端**，启动ros内核输入命令: 

```bash
roscore
```

2. **开启新终端**, 启动电机：cd到ros工作空间目录下（这里以catkin_ws目录为例），先source, 后启动nimservos_controller软件包 输入命令:
```bash
cd ~/catkin_ws
sudo su
source devel/setup.bash
roslaunch nimservos_controller nimservos_controller.launch #执行此命令报错，均为通信问题，检查电机是否正常上电，can总线是否正常
```

3. **开启新终端**, cd到ros工作空间目录下（这里以catkin_ws目录为例）, 先source, 后启动user_pkg软件包 输入命名:

```bash
cd ~/catkin_ws
sudo su
source devel/setup.bash
rosrun user_pkg main.py
```
* 如果一直提示，数据异常，则检查动捕工程启动是否正常？ 可以使用plotjuggler来确认动捕数据流是否正常
* 运行完rosrun user_pkg main.py后会出现UI控制小窗口,在UI小窗口中监听键盘输入
* o/p键用于模式切换 o->键盘控制 p->自动跟踪模式
- 在键盘控制模式下
1. w/s分别控制天轨横梁前后移动（天轨的y方向）
2. a/d分别控制天轨小车左右移动 (天轨的x方向)
注意:
天轨的坐标系为，控制台到窗户方向为X轴正方向，控制台到大实验室方向为Y轴正方向。
## 再次注意：
* 按下P键后，吊车会直接运动到机器人正上方 请在合适的时机启动 `P`自动追踪模式  
* 机器人出现意外后，停掉机器人后，按`O`键 退出自动追踪模式，否则吊车仍然会跟踪机器人运动  
## 意外情况：
出现意外不可控情况，优先大力关断**红色急停按钮**   


## 安装依赖（正常情况已安装）
```bash
pip install pynput  
sudo apt-get install python3-tk
```



# 提示： 当前NUC， 两个工程均已成功存在 `~/ZLB/catkin_ws`目录下。 vscode 直接启动两个工程即可，无需重新部署  
0. 先启动roscore
1. 在动捕windows启动sdk流播放后 ，先启动 `~/ZLB/catkin_ws/motioncapture`工程    
2. 启动天轨，先启动电机节点，在启动服务节点...