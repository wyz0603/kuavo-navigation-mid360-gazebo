#!/usr/bin/env python3
# -*- coding: UTF-8 -*-


class Calibrator(object):
    """ 校正器 """
    
    _instance = None
    def __new__(cls, *args, **kwargs):
        """ 单实例模式 """
        if not cls._instance:
            cls._instance = super().__new__(cls, *args, **kwargs)
        return cls._instance
    
    
    def __init__(self) -> None:
        pass

    
    # TODO
    # 校正执行器位置到规划器位置
