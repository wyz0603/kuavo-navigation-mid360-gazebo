# EB03 音频配置目录说明

## 目录用途

本目录保存 `EB03_LJ` USB 声卡的当前配置方案，主要解决：

- `root` 或 `gdm` 抢占 USB 声卡
- PulseAudio 多实例和残留运行时目录
- 用户启动后默认输入输出没有切到 `EB03_LJ`

目录位置：

```text
tools/audio_config/eb03_audio
```

## 当前处理逻辑

当前方案采用“双阶段判断”：

- 第一阶段：用 `/proc/asound/cards` 或 `aplay -l` 确认 `EB03_LJ` 在 ALSA 层已经出现
- 第二阶段：用 `pactl` 确认 PulseAudio 已经生成对应的 `card`、`sink`、`source`

这样做是因为：

- ALSA 能看到卡，不代表 PulseAudio 已经接管
- PulseAudio 看不到卡，也不代表 USB 声卡真的不存在

修复脚本现在会在 ALSA 已经看到 `EB03_LJ` 之后，持续检查 PulseAudio 侧是否已经准备好；如果 PulseAudio 还没完整接管，会按间隔尝试重启用户态 PulseAudio，随后再把默认输入输出切到 `EB03_LJ`。

## 主要文件

### `one_click_setup_eb03.sh`

一键部署脚本。

主要动作：

- 安装 `~/audio/set_pulse_default_eb03.sh`
- 安装 `~/.config/systemd/user/set-pulse-default-eb03.service`
- 安装 `~/.config/systemd/user/set-pulse-default-eb03.timer`
- 写入 `~/.config/pulse/client.conf`
- 修改 `~/.config/pulse/default.pa`
- 写入隐藏版 `~/.config/autostart/pulseaudio.desktop`
- 清理 `root`、`gdm`、当前用户残留的 `pulseaudio`
- 启用用户态 `pulseaudio.socket` 和 timer

### `set_pulse_default_eb03.sh`

默认设备修复脚本。

作用：

- 检查 PulseAudio 是否可连接
- 检查 ALSA 层是否已经识别到 `EB03_LJ`
- 必要时尝试启动用户态 PulseAudio
- 如果 ALSA 已识别到 `EB03_LJ`，但 PulseAudio 仍未完整接管，则按间隔重启用户态 PulseAudio 重新探测
- 等待包含 `EB03_LJ` 关键字的 `card`、`sink`、`source`
- 将默认输出和默认输入设置到目标 USB 声卡

### `set-pulse-default-eb03.service`

用户态一次性服务模板。

作用：

- 执行修复脚本

### `set-pulse-default-eb03.timer`

用户态 timer 模板。

作用：

- 在用户启动后延迟触发修复服务

### `README_audio_setup.md`

详细操作文档。

## 推荐使用方式

首次部署：

```bash
cd /home/lab/kuavo_ros_application/tools/audio_config/eb03_audio
sudo ./one_click_setup_eb03.sh
sudo reboot
```

指定目标用户部署：

```bash
cd /home/lab/kuavo_ros_application/tools/audio_config/eb03_audio
sudo TARGET_USER=leju_kuavo ./one_click_setup_eb03.sh
sudo reboot
```

## 使用注意事项

- 一键部署脚本必须以 `root` 或 `sudo` 方式运行
- 实际安装目标用户优先级为：`TARGET_USER` > `SUDO_USER` > 当前执行用户
- 安装后的执行入口是目标用户家目录下的 `~/audio/set_pulse_default_eb03.sh`
- 当前方案仍然会修改用户态 PulseAudio 配置文件，不是“纯无侵入”方案

## 核心文件

如果只关注当前方案，至少包括这 5 个文件：

```text
one_click_setup_eb03.sh
set_pulse_default_eb03.sh
set-pulse-default-eb03.service
set-pulse-default-eb03.timer
README_audio_setup.md
```

## 补充说明

- 当前方案是在现有 PulseAudio 补丁逻辑基础上，补上了对 `root` 和 `gdm` 抢占声卡的清理。
- 如果 `aplay -l` 能看到 `EB03_LJ`，但 `pactl list short cards` 看不到，通常说明 PulseAudio 还没有完成接管，而不是 USB 卡本身消失。
- 详细部署和验收步骤请查看同目录下的 `README_audio_setup.md`。
