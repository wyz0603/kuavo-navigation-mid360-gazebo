# 检查头部 NUC DHCP配置
- 将本文档中的脚本拷贝进头部 NUC
- ssh 远程进入头部 NUC
- 检查当前头部 NUC DHCP 的配置，执行以下命令：
```bash
    ./get_dhcp_range.sh
```
- 应有如下输出：
```bash
    '192.168.26.12 192.168.26.12'
```
- 如果输出：
```bash
    '错误: DHCP 配置文件 /etc/dhcp/dhcpd.conf 不存在！请联系技术支持操作！'
    请联系技术支持操作。
```
- 否则执行以下命令，修改dhcp配置：
```bash
    ./set_dhcp_range.sh && sudo systemctl restart isc-dhcp-server
```
- 再次检查头部 NUC DHCP 的配置，执行以下命令：
```bash
    ./get_dhcp_range.sh
```