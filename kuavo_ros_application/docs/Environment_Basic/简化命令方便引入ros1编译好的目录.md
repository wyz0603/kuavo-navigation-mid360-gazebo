# sros1 简化引入ros1工作空间的命令
* （1）配置一下bashrc
```bash
vim ~/.bashrc
```
* （2）在打开的文件末尾添加如下指令
```bash
# sros1
alias sros1='source ~/ros_catkin_ws/install_isolated/local_setup.bash'
```
* （3）使能一下bashrc文件
```bash
source ~/.bashrc
```

### 以后每次在 自己的工作空间下 引入 ros1源码的目录的时候，只需要在命令行下输入 sros1 即可生效
```bash
kuavo@kuavo-NUC12WSKi7:~/catkin_ws$ sros1
```

