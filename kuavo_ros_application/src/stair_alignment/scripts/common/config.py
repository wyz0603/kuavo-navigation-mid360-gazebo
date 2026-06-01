#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json

class Config:
    def __init__(self, filename):
        # 从文件加载配置
        with open(filename, 'r') as file:
            self.__dict__.update(json.load(file))
    def to_json(self):
        """
        将当前配置转换为JSON字符串。
        :return: JSON格式的字符串表示配置。
        """
        return json.dumps(self.__dict__, indent=4)

    def save_to_file(self, filename):
        """
        将当前配置保存到JSON文件。
        :param filename: 要保存配置的JSON文件路径。
        """
        with open(filename, 'w') as file:
            json.dump(self.__dict__, file, indent=4)


if __name__ == '__main__':
    # 从文件加载配置
    config = Config('config.json')
    print(config)  # 输出配置对象
    print(f"Tag ID: {config.tag_id}")
    print(f"Stand Params: {config.stand_params}")