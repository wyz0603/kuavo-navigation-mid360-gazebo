# CPU内核隔离配置说明

- 目前controller支持通过ROS参数设置隔离CPU并将主控线程绑定到隔离核心。
> 只有同时配置到了grub中隔离cpu核心和在luanch中也设置`isolated_cpus`参数，controller才会真正隔离cpu核心并绑定主控线程到隔离核心。

## 1. 通过GRUB配置设置CPU内核隔离

### 1.1 修改GRUB配置文件

使用nano编辑器打开grub配置文件：
```bash
sudo nano /etc/default/grub
```

### 1.2 添加isolcpus参数

找到 `GRUB_CMDLINE_LINUX_DEFAULT` 或 `GRUB_CMDLINE_LINUX`，在其参数中添加 `isolcpus`相关的参数(如果存在则添加到已有的`isolcpus=`后面即可)， 目前建议绑定到`2,3`两个核心：

```bash
# 示例：隔离CPU 2和3
GRUB_CMDLINE_LINUX_DEFAULT="quiet splash isolcpus=2,3"

# 或者使用范围表示法
GRUB_CMDLINE_LINUX_DEFAULT="quiet splash isolcpus=2-3"

# 或者混合表示法
GRUB_CMDLINE_LINUX_DEFAULT="quiet splash isolcpus=1,3,5-7"
```

### 1.3 保存并更新GRUB（重要！！）

保存文件（Ctrl+O，然后Enter），退出nano（Ctrl+X），然后更新grub：
```bash
sudo update-grub
```

### 1.4 重启系统（重要！！）

重启系统使配置生效：
```bash
sudo reboot
```

### 1.5 验证配置

重启后，检查isolcpus参数是否生效：
```bash
grep isolcpus /proc/cmdline
```

应该能看到类似输出：`isolcpus=2,3`

## 2. 通过ROS参数设置隔离CPU

在launch文件或参数文件中设置 `/isolated_cpus` 参数：
- 实物launch为`src/humanoid-control/humanoid_controllers/launch/load_kuavo_real.launch`, 修改确认其中添加了`isolated_cpus`相关param

```xml
<!-- 在launch文件中 -->
<rosparam param="isolated_cpus">[2, 3]</rosparam>
```

## 注意事项
- 确认是否隔离成功：运行程序终端打印中有类似`成功设置CPU亲和性到隔离核心`的输出。

