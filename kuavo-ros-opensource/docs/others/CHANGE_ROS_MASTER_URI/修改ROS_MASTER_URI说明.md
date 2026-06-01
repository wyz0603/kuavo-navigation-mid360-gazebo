# 修改 ROS_MASTER_URI 配置：

## 1. 修改躯干 NUC 配置：
1. ssh 远程进入躯干 NUC ,并进入 root 用户模式:
```bash
    sudo su
```
2. 获取当前与头部 NUC 连接的网口的 ip:
- 执行以下命令:
```bash
    ./get_ip.sh
```
- 会有如下输出：
```bash
   192.168.26.12    #复制下来，在头部 NUC 执行命令 ./add_ros_master_hosts.sh 时会用到
   #如果该脚本没有看到 IP，请联系技术支持。
```

3. 修改 hosts 文件，执行以下命令：
```bash
    ./add_ros_master_hosts.sh
```
4. 修改环境变量，执行以下命令：
```bash
    ./add_ros_master_uri.sh body  &&  source ~/.bashrc
```
5. 查看当前的 ROS_MASTER_URI 是否正确：
```bash
    echo $ROS_MASTER_URI
```
6. 输出应该如下所示:
``` bash
    http://kuavo_master:11311

```
## 2. 修改头部 NUC 配置：
1. ssh 远程进入头部 NUC，在 kuavo 用户下执行下列命令。
2. 修改 hosts 文件，执行以下脚本：
```bash
    ./add_ros_master_hosts.sh 192.168.26.12 #后面的参数 IP 由躯干操作的第二步获取。
```

3. 修改环境变量，执行以下命令：
```bash
    ./add_ros_master_uri.sh && source ~/.bashrc 
```

4. 查看当前的 ROS_MASTER_URI 是否正确：
```bash
    echo $ROS_MASTER_URI
```
输出应该如下所示:
``` bash
    http://kuavo_master:11311

```
