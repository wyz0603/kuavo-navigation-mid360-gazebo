#!/bin/bash

# 获取 mmcli 中的所有 5G 模块网络接口（如 wwan0）
modem_net_ifaces=$(mmcli -L | grep -oP '/Modem/\d+' | while read -r modem; do
    mmcli -m "${modem##*/}" 2>/dev/null | grep -Po '(\w+)\s+\(net\)' | awk '{print $1}'
done)

# 转换为数组
readarray -t modem_if_array <<< "$modem_net_ifaces"

# 获取所有默认路由
mapfile -t routes < <(ip route show | grep "^default")

if [ ${#routes[@]} -eq 0 ]; then
    echo "未找到默认路由。"
    exit 1
fi

echo "当前默认路由如下："
for i in "${!routes[@]}"; do
    route="${routes[$i]}"
    dev=$(echo "$route" | awk '/dev/ {print $5}')
    label=""
    for iface in "${modem_if_array[@]}"; do
        if [ "$dev" == "$iface" ]; then
            label=" [5G模块]"
            break
        fi
    done
    echo "$((i+1)). $route$label"
done

# 用户选择
read -p "请输入要使用的默认路由编号（1-${#routes[@]}）: " choice

if ! [[ "$choice" =~ ^[0-9]+$ ]] || [ "$choice" -lt 1 ] || [ "$choice" -gt "${#routes[@]}" ]; then
    echo "无效的选择。"
    exit 1
fi

selected_route="${routes[$((choice-1))]}"
echo "你选择了：$selected_route"

via=$(echo "$selected_route" | awk '/via/ {print $3}')
dev=$(echo "$selected_route" | awk '/dev/ {print $5}')

# 判断是否为5G模块接口
is_5g_module_iface=false
for iface in "${modem_if_array[@]}"; do
    if [ "$dev" == "$iface" ]; then
        is_5g_module_iface=true
        break
    fi
done

if $is_5g_module_iface; then
    echo "✅ 接口 $dev 是 5G 模块的网络接口"
else
    echo "⚠️ 接口 $dev 不是来自 5G 模块"
fi

# 用户输入新的 metric
read -p "请输入新的优先级（值越小，则优先级越高）: " new_metric

# 修改路由
sudo ip route del default via "$via" dev "$dev"
sudo ip route add default via "$via" dev "$dev" metric "$new_metric"

echo "已将默认路由 [$via via $dev] 的 metric 修改为 $new_metric"

