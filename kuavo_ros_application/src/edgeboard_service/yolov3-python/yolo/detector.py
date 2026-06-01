"""detector definition"""
import time
import os
import numpy as np
from .preprocess import Preprocess
import sys
upper_level_path = os.path.abspath(os.path.join(os.path.dirname(__file__), "../lib"))
sys.path.append(upper_level_path)
from libyolov3_bind import YoloV3Predictor


class YoloV3(object):
    """PPNC detector class. """
    def __init__(self, config, infer_yml):
        """init

        Args:
            config (str): path of config.json
            infer_yml (str): path od infer_cfg.yml
        """
        self.config = config
        self.model = YoloV3Predictor(config)
        self.preprocessor = Preprocess(infer_yml)
        self.total_time = 0

    def predict_image(self, img):
        """ " predict image"""
        img, size = self.preprocessor(img)
        results = self.model.predict(img, size)
        return results
    
    def predict_profile(self, img):
        """ " predict image with profile"""
        time0 = time.time()
        img, size = self.preprocessor(img)
        results = self.model.predict(img, size)
        time1 = time.time()
        self.total_time = time1 - time0
        return results
