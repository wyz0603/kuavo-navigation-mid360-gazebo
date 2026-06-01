import argparse
import os
import sys
import cv2
import json
import yaml
import logging
import pickle
import paddle
from paddle import fluid
import warnings

warnings.filterwarnings("ignore")

parent_path = os.path.abspath(os.path.join(__file__, *([".."] * 2)))
sys.path.insert(0, parent_path)
from yolo import YoloV3, vis

log_formatter = "%(levelname)s %(asctime)s: %(message)s"
logging.basicConfig(level=logging.INFO, format=log_formatter)

from paddle_postprocess import Postprocess, paddle_vis
from paddle_preprocess import Preprocess


class PaddleDetector:
    """PaddleDetector class"""

    def __init__(self, model_dir, infer_yml):
        self.postprocessor = Postprocess()
        self.preprocessor = Preprocess(infer_yml)
        self.model_dir = model_dir
        self.model_file = None
        self.params_file = None
        for file in os.listdir(model_dir):
            if file.endswith(".pdmodel"):
                self.model_file = file
            elif file.endswith(".pdiparams"):
                self.params_file = file
        assert self.model_file is not None, "pdmodel file does not exsit."
        assert self.params_file is not None, "pdiparams file does not exist."

    def predict(self, input_dict):
        exe = fluid.Executor(fluid.CPUPlace())
        [paddle_prog, feed, fetch] = fluid.io.load_inference_model(
            self.model_dir,
            exe,
            model_filename=self.model_file,
            params_filename=self.params_file,
        )

        res_paddle = exe.run(
            paddle_prog, feed=input_dict, fetch_list=fetch, return_numpy=False
        )
        boxes = res_paddle[0].__array__()
        boxes_num = res_paddle[1].__array__()
        return {"boxes": boxes, "boxes_num": boxes_num}

    def predict_image(self, img):
        """predict a image with preprocess and postprocess"""
        """ " predict image"""
        inputs = self.preprocessor(img)
        res = self.predict(inputs)
        res = self.postprocessor(res)
        return res


def argsparser():
    """
    parse command arguments
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
        "--model_dir",
        type=str,
        default="./model",
        help=("path of pdmodel and pdiparams"),
    )
    parser.add_argument(
        "--test_dir",
        type=str,
        default="./test_images",
        help="Dir of test image file",
    )
    parser.add_argument(
        "--output_dir", type=str, default="./output_dir", help="output dir."
    )
    return parser


def main(args):
    """main"""
    # get test_images
    test_dir = args.test_dir
    assert test_dir is not None, "test_dir must be provided."
    assert os.path.exists(test_dir), "test_dir does not exist."

    # check output_dir
    output_dir = args.output_dir
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
    ppnc_output_image_dir = os.path.join(output_dir, "ppnc_result_images")
    if not os.path.exists(ppnc_output_image_dir):
        os.makedirs(ppnc_output_image_dir)
    ppnc_output_pickle_dir = os.path.join(output_dir, "ppnc_result_pickle")
    if not os.path.exists(ppnc_output_pickle_dir):
        os.makedirs(ppnc_output_pickle_dir)

    paddle_output_image_dir = os.path.join(output_dir, "paddle_result_images")
    if not os.path.exists(paddle_output_image_dir):
        os.makedirs(paddle_output_image_dir)
    paddle_output_pickle_dir = os.path.join(output_dir, "paddle_result_pickle")
    if not os.path.exists(paddle_output_pickle_dir):
        os.makedirs(paddle_output_pickle_dir)

    # initial PPNC Detector
    config = args.config
    assert os.path.exists(config), "config does not exist."

    infer_yml = args.infer_yml
    assert os.path.exists(infer_yml), "infer_yml does not exist."
    with open(infer_yml, "r") as f:
        infer_yml = yaml.safe_load(f)

    ppnc_detector = YoloV3(config, infer_yml)

    # initial paddle Detector
    model_dir = args.model_dir
    assert os.path.exists(model_dir), "model does not exist."
    paddle_detector = PaddleDetector(model_dir, infer_yml)

    # infer and save result
    for test_file in os.listdir(test_dir):
        print(test_file)
        test_path = os.path.join(test_dir, test_file)
        test_image = cv2.imread(test_path)
        ppnc_result = ppnc_detector.predict_image(test_image)
        paddle_result = paddle_detector.predict_image(test_image)

        # save pickle result
        with open(
            os.path.join(ppnc_output_pickle_dir, test_file.split(".")[0] + ".pkl"),
            "wb",
        ) as ppnc_pickle_file:
            pickle.dump(ppnc_result, ppnc_pickle_file)

        with open(
            os.path.join(paddle_output_pickle_dir, test_file.split(".")[0] + ".pkl"),
            "wb",
        ) as paddle_pickle_file:
            pickle.dump(paddle_result, paddle_pickle_file)

        # save visualize result
        paddle_visual = paddle_vis(test_image, paddle_result)
        ppnc_visual = vis(test_image, ppnc_result)
        cv2.imwrite(os.path.join(paddle_output_image_dir, test_file), paddle_visual)
        cv2.imwrite(os.path.join(ppnc_output_image_dir, test_file), ppnc_visual)

    logging.log(
        logging.INFO, "paddle pickle results saved in %s", paddle_output_pickle_dir
    )
    logging.log(
        logging.INFO, "paddle visualize results saved in %s", paddle_output_image_dir
    )
    logging.log(logging.INFO, "ppnc pickle results saved in %s", ppnc_output_pickle_dir)
    logging.log(
        logging.INFO, "ppnc visualize results saved in %s", ppnc_output_image_dir
    )


if __name__ == "__main__":
    paddle.enable_static()
    parser = argsparser()
    args = parser.parse_args()
    main(args)
