import json
import os
import argparse
import sys
import yaml
import numpy as np
import warnings
import cv2
warnings.filterwarnings("ignore")


parent_path = os.path.abspath(os.path.join(__file__, *([".."] * 2)))
sys.path.insert(0, parent_path)
from yolo import YoloV3, vis


def argsparser():
    """
    解析命令行参数，配置模型参数和优化器等参数
    """
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--config",
        type=str,
        default="./model/config.json",
        help=("path of deploy config.json"),
    )
    parser.add_argument(
        "--infer_yml",
        type=str,
        default="./model/infer_cfg.yml",
        help=("path of infer_cfg.yml"),
    )
    parser.add_argument(
        "--test_image",
        type=str,
        default="./test_images/000000025560.jpg",
        help="Path of test image file.",
    )
    parser.add_argument(
        "--visualize", action="store_true", help="whether to visualize."
    )
    parser.add_argument(
        "--with_profile", action="store_true", help="whether to predict with profile."
    )
    return parser


def main(args):
    """main"""
    # init YoloV3
    config = args.config
    assert os.path.exists(config), "deploy_config does not exist."
    
    infer_yml = args.infer_yml
    assert os.path.exists(infer_yml), "infer_yml does not exist."
    with open(infer_yml, "r") as f:
        infer_yml = yaml.safe_load(f)
        
    detector = YoloV3(config, infer_yml)
        
    test_image = args.test_image
    image = cv2.imread(test_image)
    
    with_profile = args.with_profile
    if with_profile:
        results = detector.predict_profile(image)
        print("total time: ", detector.total_time)
    else:
        results = detector.predict_image(image)
    # visualize
    visualize = args.visualize
    if visualize:
        render_img = vis(image, results)
        cv2.imwrite("./vis.jpg", render_img)
        print("visualize result saved as vis.jpg.")

if __name__ == "__main__":
    parser = argsparser()
    args = parser.parse_args()
    main(args)
