## 目录结构

```
checktools/
├── README.md                           # 本文件 - 工具集总览
├── camera_tools/                       # 相机相关工具集合
│   ├── run_camera_test.sh              # 相机测试工具启动脚
│   └── camera_test_tool.py             # 相机测试工具
```

## 🎯 工具分类

### 📹 ROS摄像头检测工具


## 🚀 快速开始


### 方法1: 使用相机测试脚本（推荐）

#### 摄像头测试
```bash
# 启动并自动安装/编译依赖，然后运行相机测试
cd tools/checktools/camera_tools
chmod +x run_camera_test.sh
./run_camera_test.sh
```

运行后脚本将启动相机，并依次订阅图像并识别 apriltag。将 apriltag 标签放到对应的相机前，终端会打印 apriltag 识别信息

