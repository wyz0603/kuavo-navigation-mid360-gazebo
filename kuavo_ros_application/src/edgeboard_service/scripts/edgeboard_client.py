# -*- coding:utf-8 -*-

from __future__ import print_function 

import sys
import rospy
from edgeboard_service.srv import * #注意是功能包名.srv

import os

# 添加路径
script_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, script_dir)

# 导入
from tts_ws_python3_demo import tts_xunfei

#中文标签
labels=[
    "人",
    "自行车",
    "汽车",
    "摩托车",
    "飞机",
    "巴士",
    "火车",
    "卡车",
    "船",
    "红绿灯",
    "消防栓",
    "停车标志",
    "停车计时器",
    "板凳",
    "鸟",
    "猫",
    "狗",
    "马",
    "羊",
    "牛",
    "大象",
    "熊",
    "斑马",
    "长颈鹿",
    "背包",
    "伞",
    "手提包",
    "领带",
    "手提箱",
    "飞盘",
    "滑雪单板",
    "滑雪双板",
    "运动球",
    "风筝",
    "棒球棒",
    "棒球手套",
    "滑板",
    "冲浪板",
    "网球拍",
    "瓶子",
    "酒杯",
    "杯子",
    "叉子",
    "刀子",
    "勺子",
    "碗",
    "香蕉",
    "苹果",
    "三明治",
    "橙子",
    "花椰菜",
    "胡萝卜",
    "热狗",
    "披萨",
    "甜甜圈",
    "蛋糕",
    "椅子",
    "沙发",
    "盆栽",
    "床上",
    "餐桌",
    "厕所",
    "显示器", # 电视
    "笔记本电脑",
    "鼠标",
    "遥控器",
    "键盘",
    "手机",
    "微波炉",
    "烤箱",
    "烤面包机",
    "水池",
    "冰箱",
    "书",
    "钟表",
    "花瓶",
    "剪刀",
    "泰迪熊",
    "吹风机",
    "牙刷",
]

def edgeboard_yolo_client(my_config):
    rospy.wait_for_service('edgeboard_yolo')
    try:
        edgeboard_yolo = rospy.ServiceProxy('edgeboard_yolo', EbMessage)
        resp1 = edgeboard_yolo(my_config)
        return resp1.result
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def main():
    # 发送请求
    my_config = "my_config"
    print("Send config:%s",my_config)
    Accept_result=edgeboard_yolo_client(my_config)
    print("Accept result:",Accept_result)
    # 字符串处理
    strlist = Accept_result.split(' ') # 用逗号分割str字符串，并保存到列表
    # 根据count判断是label还是num
    count=0

    for value in strlist: # 循环输出列表值
        if value.isdigit():
            if(count==0):
                count=1
                label_cn = int(value)
                #print(num)
            else :
                count=0
                text = "我看到" + str(value) + "个" + str(labels[label_cn])
                print(text)
                # 调用TTS
                tts_xunfei(text)
                os.system("play -q output.mp3")
        else:
            print("end")

if __name__ == "__main__":
    main()
