# EB03 音频配置说明

## 目标

机器启动后，将 USB 声卡 `EB03_LJ` 同时作为默认播放设备和默认录音设备。

正常情况下，`pactl info` 应显示：

```text
Default Sink: alsa_output.usb-ShengZhi_EB03_LJ_*.analog-stereo
Default Source: alsa_input.usb-ShengZhi_EB03_LJ_*.analog-stereo
```

说明：

- USB 设备名中的序列号部分可能变化，因此文档中统一用 `*` 表示。
- 只要 `card`、`sink`、`source` 名称中包含 `EB03_LJ`，修复脚本就会识别为目标设备。

## 当前方案解决的问题

当前方案主要解决这几类问题：

- `gdm` 先启动并抢占 USB 声卡
- `root` 身份误启动 `pulseaudio`，导致用户会话拿不到声卡
- PulseAudio 多实例或残留运行时目录导致连接异常
- USB 声卡启动时序慢于用户会话，导致默认设备恢复失败
- 默认输出和默认输入被自动切回板载声卡

## 当前处理逻辑

当前方案不是单纯“看到 `pactl` 里有卡就设置默认设备”，而是分成两层判断：

1. 先确认 ALSA 层已经识别到 `EB03_LJ`
2. 再确认 PulseAudio 层已经生成对应的 `card`、`sink`、`source`

这样设计的原因是：

- `aplay -l` 或 `/proc/asound/cards` 能看到 `EB03_LJ`，只说明内核已经识别到 USB 声卡
- `pactl list short cards/sinks/sources` 能看到目标对象，才说明 PulseAudio 已经真正接管，后续 `set-default-sink` 和 `set-default-source` 才有意义

也就是说：

- 只看 `pactl` 不够，因为它可能暂时还没接管到 USB 卡
- 只看 `aplay` 也不够，因为 ALSA 有卡不代表 PulseAudio 已经生成可用 `sink/source`

当前修复脚本的处理顺序是：

1. 确保用户态 PulseAudio 可连接
2. 检查 ALSA 层是否存在包含 `EB03_LJ` 的声卡
3. 检查 PulseAudio 层是否存在包含 `EB03_LJ` 的 `card`
4. 检查是否已经生成包含 `EB03_LJ` 的 `sink` 和真实 `source`
5. 如果 ALSA 已有卡、但 PulseAudio 仍未正确接管，则继续等待，并按重试间隔尝试重启用户态 PulseAudio 重新探测
6. 目标 `sink/source` 出现后，设置为默认输入输出

这里的“仍未正确接管”包括三种情况：

- `ALSA` 已看到 `EB03_LJ`，但 `pactl list cards short` 里还没有对应 `card`
- `PulseAudio card` 已出现，但还没有对应 `sink`
- `PulseAudio card` 已出现，但还没有对应真实 `source`

也就是说，当前版本已经改成：

- 只要 ALSA 层已经识别到 `EB03_LJ`
- 且 PulseAudio 侧还没完整准备好
- 就允许脚本周期性重启一次用户态 PulseAudio，推动重新探测

## 当前脚本实际做的事情

`one_click_setup_eb03.sh` 会执行以下动作：

1. 创建 `~/audio`、`~/.config/systemd/user`、`~/.config/pulse`、`~/.config/autostart`
2. 安装修复脚本、README、user systemd service、timer
3. 写入 `~/.config/pulse/client.conf`：
   ```text
   autospawn = no
   ```
4. 写入 `~/.config/autostart/pulseaudio.desktop`，屏蔽桌面自启动
5. 修改用户态 `~/.config/pulse/default.pa`
6. 执行 `loginctl enable-linger`
7. 清理冲突的 `pulseaudio` 进程：
   - `root`
   - `gdm`
   - 当前目标用户残留实例
8. 启用：
   - `pulseaudio.socket`
   - `set-pulse-default-eb03.timer`
9. 立即执行一次默认设备修复脚本

其中第 7 步的目的，是优先释放被其他会话抢占的 USB 声卡，避免后续用户态 PulseAudio 只能看到板载声卡。

## 当前脚本中的关键补丁点

安装后，用户态 `~/.config/pulse/default.pa` 会被调整为：

- `load-module module-udev-detect`
  改为
  `load-module module-udev-detect use_ucm=0`
- 注释掉 `module-switch-on-port-available`
- 注释掉 `module-switch-on-connect`
- 注释掉 `module-default-device-restore`

这样做的目的是：

- 减少自动切换默认设备的干扰
- 尽量让 `EB03_LJ` 被稳定识别

## 目录内文件

一键部署脚本：

```text
tools/audio_config/eb03_audio/one_click_setup_eb03.sh
```

默认设备修复脚本模板：

```text
tools/audio_config/eb03_audio/set_pulse_default_eb03.sh
```

用户服务模板：

```text
tools/audio_config/eb03_audio/set-pulse-default-eb03.service
tools/audio_config/eb03_audio/set-pulse-default-eb03.timer
```

## 安装后文件位置

默认安装目标为执行用户的家目录，目标用户规则如下：

- 优先使用环境变量 `TARGET_USER`
- 否则使用 `sudo` 调用者 `SUDO_USER`
- 再否则使用当前执行用户

安装完成后，关键文件位于：

```text
~/audio/set_pulse_default_eb03.sh
~/audio/README_audio_setup.md
~/.config/systemd/user/set-pulse-default-eb03.service
~/.config/systemd/user/set-pulse-default-eb03.timer
~/.config/pulse/default.pa
~/.config/pulse/default.pa.bak
~/.config/pulse/client.conf
~/.config/autostart/pulseaudio.desktop
```

其中：

- `default.pa.bak` 是首次部署时自动保存的原始备份
- service 中的 `ExecStart` 会被改写为安装后的真实路径

## 一键部署

首次部署执行：

```bash
cd /home/lab/kuavo_ros_application/tools/audio_config/eb03_audio
sudo ./one_click_setup_eb03.sh
```

如需明确指定目标用户：

```bash
cd /home/lab/kuavo_ros_application/tools/audio_config/eb03_audio
sudo TARGET_USER=leju_kuavo ./one_click_setup_eb03.sh
```

建议部署完成后重启：

```bash
sudo reboot
```

## 手动执行修复

如果只想重新设置默认设备，可执行：

```bash
~/audio/set_pulse_default_eb03.sh
```

也可以指定关键字：

```bash
~/audio/set_pulse_default_eb03.sh EB03_LJ
```

可选环境变量：

```bash
WAIT_SECONDS=60 ~/audio/set_pulse_default_eb03.sh
```

说明：

- `WAIT_SECONDS` 默认值为 `20`
- `RESTART_INTERVAL_SECONDS` 默认值为 `15`
- 脚本会先确认 PulseAudio 可连接
- 然后先确认 ALSA 层存在 `EB03_LJ`
- 再等待 `EB03_LJ` 对应的 `card`、`sink`、`source` 同时出现
- 只要 ALSA 已看到 `EB03_LJ`，但 PulseAudio 侧还没完整准备好，脚本会按 `RESTART_INTERVAL_SECONDS` 周期尝试重启一次用户态 PulseAudio
- 如果立即执行时 USB sink 还没出现，通常要依赖登录后的 timer 补一次修复

## 常用命令

查看默认输入输出：

```bash
pactl info
```

查看设备枚举结果：

```bash
pactl list short cards
pactl list short sinks
pactl list short sources
```

重启修复服务：

```bash
systemctl --user restart set-pulse-default-eb03.service
```

查看用户服务状态：

```bash
systemctl --user status pulseaudio.socket set-pulse-default-eb03.service set-pulse-default-eb03.timer --no-pager
```

查看最近日志：

```bash
journalctl --user -u set-pulse-default-eb03.service --no-pager -n 100
```

查看当前 PulseAudio 进程：

```bash
ps -ef | grep [p]ulseaudio
```

检查 USB 声卡是否被别人抢占：

```bash
fuser -v /dev/snd/pcmC0D0p
fuser -v /dev/snd/pcmC0D0c
```

如果当前 shell 不是目标用户自己的登录会话，可用下面方式执行检查命令：

```bash
sudo -u leju_kuavo XDG_RUNTIME_DIR=/run/user/$(id -u leju_kuavo) DBUS_SESSION_BUS_ADDRESS=unix:path=/run/user/$(id -u leju_kuavo)/bus pactl info
```

## 验证清单

### 1. 用户服务状态

执行：

```bash
systemctl --user status pulseaudio.socket set-pulse-default-eb03.timer --no-pager
```

期望结果：

- `pulseaudio.socket` 已启用
- `set-pulse-default-eb03.timer` 已启用且为 `active`

### 2. 默认设备检查

执行：

```bash
pactl info
```

期望结果：

- `Default Sink` 指向包含 `EB03_LJ` 的 USB sink
- `Default Source` 指向包含 `EB03_LJ` 的 USB source

### 3. PulseAudio 对象检查

执行：

```bash
pactl list short cards
pactl list short sinks
pactl list short sources
```

期望结果：

- `cards` 中存在包含 `EB03_LJ` 的声卡
- `sinks` 中存在包含 `EB03_LJ` 的输出设备
- `sources` 中存在包含 `EB03_LJ` 的真实输入设备

### 4. ALSA 硬件检查

执行：

```bash
cat /proc/asound/cards
aplay -l
```

期望结果：

- 能看到包含 `EB03_LJ` 的 USB 声卡

如果这里能看到 `EB03_LJ`，但 `pactl list short cards` 看不到，通常说明：

- PulseAudio 还没有接管到 USB 卡
- 或者 USB 卡曾被 `root` / `gdm` / 残留 PulseAudio 实例抢占
- 此时应优先检查用户态 `pulseaudio` 状态、timer 是否触发，以及设备文件是否被占用

### 5. 抢占排查

执行：

```bash
fuser -v /dev/snd/pcmC0D0p
fuser -v /dev/snd/pcmC0D0c
ps -ef | grep [p]ulseaudio
```

重点关注：

- 是否存在 `gdm` 的 `pulseaudio`
- 是否存在 `root` 的 `pulseaudio`
- 是否存在多个用户态 `pulseaudio` 实例

如果发现不是目标用户自己的 PulseAudio 在占用设备，优先清理冲突进程后再重新执行修复脚本。

执行：

```bash
aplay -l
arecord -l
cat /proc/asound/cards
```

期望结果：

- ALSA 能看到 `EB03_LJ`
- PulseAudio 最终也能看到对应 USB `card`/`sink`/`source`
