# nimservos SDK 说明:
    电机型号:PMM6040 PMMP6040
    通讯协议:canopen
    设备支持:bm USB-CAN
    2024/8/1: 
            完成SDO模式下的canopen message 发送
            CiA402运动模式（包括 轮廓位置模式(PP) 速度模式(VM) 轮廓速度模式(PV) 轮廓转矩模式(PT) 原点回归模式(HM) 插补模式(IP) 循环同步位置模式(CSP) 循环同步速度模式(CSV) 循环同步转矩模式(CST)）
    2024/8/9:
            添加读速度和读位置请求API接口
            添加id设置
            成功跑通CSP CSV CST 模式
                note:CSV模式下要在上位机修改故障动作配置,修改位置超差错误故障码(改为不记录),修改曲线规划参数过小故障(改为不记录),然后保存
    2024/8/12:
            修改调用逻辑,整个sdk大致分为初始化 初始化模式 刷新目标值
# SDK使用方式:
从nimservosSDK 导入 NiMServos
NiMServos()会初始化usb-can设备
调用NiMServos中的方法

# SDK测试方式
sudo ./nimservos_test.sh