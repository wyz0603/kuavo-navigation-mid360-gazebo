#!/usr/bin/env python
# coding=utf-8

# from lejulib import *
import os

# from lejulib import *
clear_function = {}
from model_utils import *

def get_model_path():
    current_directory = os.path.dirname(os.path.abspath(__file__))
    model_name = "best.pt"
    model_path = current_directory + '/' + model_name
    print(f"model path: {model_path}")
    return model_path

def main():
    yolo_detection = YOLO_detection()
    yolo_detection.init_ros_node()

    try:
        model = yolo_detection.load_model(get_model_path())
        detections = yolo_detection.get_detections("head", model)
        obj = yolo_detection.get_max_area_object(detections)
        if obj['x'] >= 20:
            print(f"walk forward: {obj}")
            # client_walk.slow_walk('forward', 1)
        while True:
            detections2 = yolo_detection.get_detections("head", model)
            obj2 = yolo_detection.get_min_area_object(detections2)
            if obj2['w'] >= 10:
                print(f"walk forward: {obj2}")
                # client_walk.slow_walk('forward', 1)
            else:
                break

    except Exception as err:
        # serror(err)
        print(f"Error: {err}")
    finally:
        for name in clear_function:
            clear_function.get(name)()

if __name__ == '__main__':
    main()
