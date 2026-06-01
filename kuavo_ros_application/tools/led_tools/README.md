# LED 硬件测试工具

这个目录包含了 Kuavo 机器人 LED 灯带的硬件直连测试工具。

## 文件说明

- `led_comprehensive_test.py` - 综合LED硬件测试脚本，支持多种测试模式
- `led_demo.py` - 简单的LED演示脚本
- `quick_test.sh` - 快速测试启动脚本

## 使用方法

### 1. LED演示脚本
```bash
cd <kuavo_ros_application>/tools/led_tools
python3 led_demo.py
```

**测试效果：**
- 🔴 **红色常亮** (2秒) - 稳定的纯红色光带
- 🟢 **绿色呼吸** (3秒) - 绿色光带缓慢呼吸变亮变暗
- 🔵 **蓝色快闪** (3秒) - 蓝色光带快速闪烁，非常醒目
- 🌈 **彩虹律动** (4秒) - 10色彩虹流水效果（红橙黄绿青蓝靛紫粉白）
- 💡 **暖白色常亮** (2秒) - 温暖柔和的暖白光带
- ⚫ **关闭LED** - 所有LED熄灭

### 2. 综合测试脚本

#### 交互式测试（推荐）
```bash
cd <kuavo_ros_application>/tools/led_tools
python3 led_comprehensive_test.py
```

**测试效果：** 提供菜单选择不同模式，可自定义颜色和持续时间，支持：
- 常亮/呼吸/快闪/律动四种模式
- 多种颜色预设（彩虹、单色、渐变等）
- 自定义RGB颜色输入

#### 完整自动测试
```bash
cd <kuavo_ros_application>/tools/led_tools
python3 led_comprehensive_test.py --test-type full
```

**测试效果：** 自动依次测试8种不同的LED效果组合：
- 常亮红色 → 呼吸绿色 → 快闪蓝色 → 律动彩虹 → 常亮白色 → 呼吸暖光 → 快闪冷光 → 常亮红蓝渐变

#### 单项测试
```bash
cd <kuavo_ros_application>/tools/led_tools

# 测试常亮红色模式
python3 led_comprehensive_test.py --test-type single --led-mode 0 --color-preset red --duration 5

# 测试呼吸彩虹模式
python3 led_comprehensive_test.py --test-type single --led-mode 1 --color-preset rainbow --duration 10
```

**测试效果：** 根据指定参数显示对应的LED效果

#### 查看所有颜色预设
```bash
cd <kuavo_ros_application>/tools/led_tools

python3 led_comprehensive_test.py --list-presets
```

### 3. 快速测试脚本（常规测试）

```bash
cd <kuavo_ros_application>/tools/led_tools

./quick_test.sh
```

**测试效果：** 快速验证LED基本功能，依次显示：
- 🔴 常亮红色 (3秒) - 验证基本显示功能
- 🟢 呼吸绿色 (3秒) - 验证呼吸模式
- 🔵 快闪蓝色 (3秒) - 验证快闪模式  
- 🌈 律动彩虹 (3秒) - 验证律动模式
- ⚫ 关闭LED (1秒) - 验证关闭功能

## LED 模式说明

- **模式 0**: 常亮模式 - LED保持设定颜色不变，亮度稳定
- **模式 1**: 呼吸模式 - LED颜色在设定值和关闭之间缓慢变化，类似呼吸效果
- **模式 2**: 快闪模式 - LED快速闪烁，闪烁频率较高
- **模式 3**: 律动模式 - LED显示动态律动效果，颜色会产生流动感

## 颜色预设说明

- **rainbow**: 彩虹渐变 - 红橙黄绿青蓝靛紫粉白的渐变色带
- **red/green/blue**: 单色光带 - 纯色稳定光线
- **white**: 纯白色光带 - 明亮的白色照明效果
- **warm**: 暖光效果 - 温暖的橙黄色调光线
- **cool**: 冷光效果 - 清冷的蓝白色调光线  
- **gradient_red_blue**: 红蓝渐变 - 从红色渐变到蓝色的过渡效果
- **off**: 关闭状态 - 所有LED熄灭

## 注意事项

1. **纯硬件模式**: 直接通过串口控制LED，无需ROS环境
2. 自动安装所需的 pyserial 库（使用清华源镜像，版本 3.5）
3. 所有测试完成后会自动关闭LED
4. 可以随时按 `Ctrl+C` 中断测试并安全关闭LED

## 依赖库说明

脚本会自动检测并安装所需的依赖库：
- **pyserial 3.5**: 用于串口通信
- **镜像源**: 使用清华大学 PyPI 镜像 (https://pypi.tuna.tsinghua.edu.cn/simple) 加速下载

如需手动安装依赖，可执行：
```bash
python3 -m pip install -i https://pypi.tuna.tsinghua.edu.cn/simple pyserial==3.5
```

## 故障排除

如果遇到设备连接问题，请检查：
1. UDEV规则是否正确配置：
   ```bash
   sudo cp ~/kuavo_ros_application/src/kuavo_led/rules/99-led.rules /etc/udev/rules.d/
   sudo usermod -a -G dialout $USER
   sudo udevadm control --reload-rules
   sudo udevadm trigger
   ```
2. `/dev/ttyLED0` 设备是否存在：
   ```bash
   ls /dev/ttyLED*
   ```
3. LED硬件是否正常连接
