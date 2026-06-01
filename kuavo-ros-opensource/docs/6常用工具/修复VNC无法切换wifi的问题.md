# 问题描述

VNC远程桌面连接时无法通过桌面右上角WIFI设置图标修改或更改WIFI

## 原因

用户没有权限。

VNC连接通过图形界面切换wifi时没有反应，网络也没有切换，此时查看NetworkManager日志
journalctl -u NetworkManager中出现audit: op="connection-add-activate" pid=1225 uid=1000 result="fail" reason="Not authorized to control networking."

## 解决办法

- 添加当前用户到netdev组：
```
sudo usermod -aG netdev $(whoami)
```
- 创建或编辑 /etc/polkit-1/localauthority/50-local.d/org.freedesktop.NetworkManager.pkla 文件。
```
sudo vim /etc/polkit-1/localauthority/50-local.d/org.freedesktop.NetworkManager.pkla
```
添加以下内容：
```
[nm-applet]
Identity=unix-group:netdev
Action=org.freedesktop.NetworkManager.*
ResultAny=yes
ResultInactive=no
ResultActive=yes
```

- 重启NetworkManager

```
sudo systemctl restart NetworkManager
```

此时可以通过VNC界面正常修改wifi。

或者可以直接执行脚本

```
cd ~/kuavo-ros-opensource
```

```
./tools/enable_vnc_network_config.sh
```