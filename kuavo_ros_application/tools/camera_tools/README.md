
- [相机序列号配置总览](#相机序列号配置总览)
  - [适用脚本与对应相机](#适用脚本与对应相机)
  - [快速使用](#快速使用)
- [set\_orbbec\_serials.sh 使用说明](#set_orbbec_serialssh-使用说明)
  - [🧩 脚本功能](#-脚本功能)
  - [🚀 使用方法](#-使用方法)
    - [1. 正常模式（自动获取序列号）](#1-正常模式自动获取序列号)
    - [2. 对调模式（跳过获取，直接交换序列号）](#2-对调模式跳过获取直接交换序列号)
  - [🔍 验证修改是否生效](#-验证修改是否生效)
  - [⚠️ 注意事项](#️-注意事项)
  - [📄 示例输出](#-示例输出)
- [set\_realsense\_wrist\_serials.sh 使用说明](#set_realsense_wrist_serialssh-使用说明)
  - [🧩 脚本功能](#-脚本功能-1)
  - [🚀 使用方法](#-使用方法-1)
  - [🔍 验证修改是否生效](#-验证修改是否生效-1)
  - [📄 补充：相机的启动方式](#-补充相机的启动方式)
  - [📄 补充：通过 rqt 查看图像的方式(需要先参考上一步启动相机)](#-补充通过-rqt-查看图像的方式需要先参考上一步启动相机)

# 相机序列号配置总览

`tools/camera_tools` 目录提供了两类相机序列号配置脚本：  
- `set_orbbec_serials.sh`：用于头部与腰部 Orbbec 相机序列号配置。  
- `set_realsense_wrist_serials.sh`：用于左右手腕 RealSense 相机序列号配置。  

两者都会将结果写入 `~/.bashrc` 环境变量。执行后请运行：
```bash
source ~/.bashrc
```

## 适用脚本与对应相机
- 头部 + 腰部（Orbbec）：
```bash
bash tools/camera_tools/set_orbbec_serials.sh
```
- 左手腕 + 右手腕（RealSense）：
```bash
bash tools/camera_tools/set_realsense_wrist_serials.sh
```
- 若通过rqt工具发现某两个相机序列号匹配反了，可为相应脚本传入 `true` 执行“对调模式”（不重新扫描，仅交换已有变量）：
```bash
bash tools/camera_tools/set_orbbec_serials.sh true
bash tools/camera_tools/set_realsense_wrist_serials.sh true
```

## 快速使用
1. 进入工作空间：`cd ~/kuavo_ros_application`
2. 根据相机类型运行对应脚本（Orbbec 或 RealSense 手腕）。
3. 执行 `source ~/.bashrc` 使变量生效。
4. 启动相机后用 `rqt_image_view` 判断画面对应关系；若不一致，再执行对应脚本的 `true` 对调模式。


# set_orbbec_serials.sh 使用说明

## 🧩 脚本功能
该脚本用于自动获取或切换 Orbbec 相机序列号（serial number），并写入到用户的 `~/.bashrc` 环境变量中。
适用于双相机系统，例如：
- 头部相机（HEAD_CAMERA）
- 腰部相机（WAIST_CAMERA）

---

## 🚀 使用方法

进入工作空间:
```bash
    cd ~/kuavo_ros_application
```
确保脚本具有可执行权限：
```bash
chmod +x tools/camera_tools/set_orbbec_serials.sh
```

### 1. 正常模式（自动获取序列号）
重新检测 Orbbec USB 设备序列号并写入到 `~/.bashrc`：

```bash
bash tools/camera_tools/set_orbbec_serials.sh
```

执行后，脚本会：
- 按 `src/OrbbecSDK_ROS1/scripts/list_ob_devices.sh` 的方式扫描 `/sys/bus/usb/devices/*`
- 筛选 `idVendor=2bc5` 的 Orbbec 设备
- 输出设备名、USB 端口、序列号
- 将检测到的前两个序列号写入：
  - `HEAD_CAMERA_SERIAL_NO`
  - `WAIST_CAMERA_SERIAL_NO`
- 提示执行：`source ~/.bashrc`

---

### 2. 对调模式（跳过获取，直接交换序列号）
如果两台相机安装方向互换，可直接交换环境变量：

```bash
bash tools/camera_tools/set_orbbec_serials.sh true
```

该模式不会重新获取序列号，而是直接在 `~/.bashrc` 中对调：
- `HEAD_CAMERA_SERIAL_NO`
- `WAIST_CAMERA_SERIAL_NO`

---

## 🔍 验证修改是否生效
执行以下命令查看结果：
```bash
echo $HEAD_CAMERA_SERIAL_NO
echo $WAIST_CAMERA_SERIAL_NO
```

---

## ⚠️ 注意事项
- 首次使用必须运行一次正常模式（即不加 `true` 参数），以确保 `.bashrc` 中有相机变量。
- 若检测到的 Orbbec 序列号不足 2 个，脚本会退出并提示检查连接。
- 是否需要对调序列号，请以 `rqt_image_view` 的实际画面为准；不一致时执行 `bash tools/camera_tools/set_orbbec_serials.sh true`。

---

## 📄 示例输出

**写入相机序列号到环境变量：**
- `bash tools/camera_tools/set_orbbec_serials.sh`
```
🔎 正在扫描 Orbbec 设备...
Found Orbbec device Gemini 335L, usb port 1-6, serial number CP2A75300033
Found Orbbec device Gemini 335L, usb port 1-5, serial number CP2A753000G2
✅ 获取到序列号:
   HEAD -> CP2A75300033
   WAIST -> CP2A753000G2
✅ 已写入 ~/.bashrc
```

**若相机序列号和实际相反，进行调换：**
- `bash tools/camera_tools/set_orbbec_serials.sh true`
```
🔁 参数为 true，跳过获取，直接对调已有变量...
✅ 已对调并写入 ~/.bashrc
```

---

# set_realsense_wrist_serials.sh 使用说明

## 🧩 脚本功能
该脚本用于自动获取或切换 RealSense 双手腕相机序列号，并写入到 `~/.bashrc` 环境变量中：
- `LEFT_WRIST_CAMERA_SERIAL_NO`
- `RIGHT_WRIST_CAMERA_SERIAL_NO`

脚本路径：
```bash
tools/camera_tools/set_realsense_wrist_serials.sh
```

## 🚀 使用方法
自动检测并写入左右手腕序列号：
```bash
bash tools/camera_tools/set_realsense_wrist_serials.sh
```

如果左右手安装互换，直接交换已有变量（不重新扫描）：
```bash
bash tools/camera_tools/set_realsense_wrist_serials.sh true
```

## 🔍 验证修改是否生效
```bash
echo $LEFT_WRIST_CAMERA_SERIAL_NO
echo $RIGHT_WRIST_CAMERA_SERIAL_NO
```

是否需要对调序列号，请以 `rqt_image_view` 的实际画面为准；不一致时执行：
```bash
bash tools/camera_tools/set_realsense_wrist_serials.sh true
```
---

## 📄 补充：相机的启动方式
```bash
cd ~/kuavo_ros_application
source devel/setup.bash
# 仅头部+腰部相机（无手腕）
roslaunch dynamic_biped kuavo5_sensor_only_enable.launch

# 头部+腰部+左右手腕相机
roslaunch dynamic_biped kuavo5_sensor_only_enable.launch enable_wrist_camera:=true
```

## 📄 补充：通过 rqt 查看图像的方式(需要先参考上一步启动相机)
- 新终端运行 `rqt_image_view`
- 在左上角选择相应图像话题进行检查
  - 头部相机 RGB 图像话题：`/camera/color/image_raw`
  - 腰部相机 RGB 图像话题：`/waist_camera/color/image_raw`
  - 左手手腕相机 RGB 图像话题(若有)：`/left_wrist_camera/color/image_raw`
  - 右手手腕相机 RGB 图像话题(若有)：`/right_wrist_camera/color/image_raw`
- 根据画面内容判断是否需要对调序列号：头/腰用 `set_orbbec_serials.sh true`，左/右手腕用 `set_realsense_wrist_serials.sh true`。
