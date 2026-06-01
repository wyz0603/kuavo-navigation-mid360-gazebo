# 修改 DHCP 主机服务

本说明文档介绍如何使用 `update_dhcp.sh` 脚本在 Linux 系统上自动配置 USB 声卡、5G 模块、DHCP 服务及 NAT 转发，实现内网设备通过指定网卡共享 5G 网络上网。

## 文档说明

- 本脚本适用于基于 Debian/Ubuntu 的系统。
- 脚本自动完成以下操作：
  1. 配置 USB 声卡的 udev 规则，确保插入后自动加载驱动。
  2. 配置 5G 模块的 udev 规则，将其命名为 `wwan0`。
  3. 自动查找以 `enx0` 开头的 USB 网卡，并将其作为 DHCP 服务的内网网卡。
  4. 安装并配置 `isc-dhcp-server`，为内网分配固定 IP 地址。
  5. 使用 NetworkManager 设置内网网卡静态地址。
  6. 配置 iptables，实现内网到 5G 网口（`wwan0`）的 NAT 转发。
  7. 检查并为其他活动网络接口（如 WiFi）自动配置 NAT 转发规则。

## 前提条件

- 需要以 root 权限运行脚本。
- 系统已联网，能够访问软件源安装必要软件包。
- 目标 USB 网卡名称以 `enx0` 开头，且仅有一个此类网卡。
- 5G 模块为 Quectel RM530N-GL，VendorID 为 `2c7c`，ProductID 为 `0801`。
- USB 声卡 VendorID 为 `1b3f`，ProductID 为 `2008`。

## 使用方法

1. 进入脚本所在目录：

   ```bash
   cd ~/kuavo-ros-control/tools/change_dhcp_host
   ```

2. 赋予脚本执行权限：

   ```bash
   chmod +x update_dhcp.sh
   ```

3. 以 root 权限运行脚本：

   ```bash
   sudo ./update_dhcp.sh
   ```
   ***中间会有一次弹框选择，按回车键选择 'yes' 即可***

4. 按照脚本提示操作，等待脚本自动完成所有配置。

## 最终结果

- 插入 USB 声卡后，系统会自动加载驱动，无需手动干预。
- 5G 模块插入后自动命名为 `wwan0`，作为外网出口。
- 内网 USB 网卡（如 `enx0xxxx`）被配置为静态 IP `192.168.26.1`，并作为 DHCP 服务的分配网卡。
- 内网设备通过 DHCP 获取 `192.168.26.12` 地址，并通过 5G 网络上网。
- 系统已自动配置 NAT 转发和必要的 iptables 规则，支持多种外部网络接口（如 5G、WiFi）。
- 所有配置在重启后依然生效。

如需自定义网段、IP 分配范围或其他参数，请根据实际需求修改 `update_dhcp.sh` 脚本中的相关变量。

## 结果验证
1. 进入 root 用户，使用命令播放语音文件：
```bash
   sudo su
   cd /home/lab/.config/lejuconfig/music/
   play 1_挥手.wav
```
2. 验证 DHCP 服务是否正常：
- 接入上位机，检查上位机的 IP 是否被分配为 192.168.26.12.