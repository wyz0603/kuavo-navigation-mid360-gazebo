"""preprocess"""
import numpy as np
import cv2


def decode_image(im_file):
    """read rgb image
    Args:
        im_file (str|np.ndarray): input can be image path or np.ndarray
        im_info (dict): info of image
    Returns:
        im (np.ndarray):  processed image (np.ndarray)
        im_info (dict): info of processed image
    """
    if isinstance(im_file, str):
        with open(im_file, "rb") as f:
            im_read = f.read()
        data = np.frombuffer(im_read, dtype="uint8")
        im = cv2.imdecode(data, 1)  # BGR mode, but need RGB mode
        im = cv2.cvtColor(im, cv2.COLOR_BGR2RGB)
    else:
        im = im_file
        im = cv2.cvtColor(im, cv2.COLOR_BGR2RGB)
    return im


class NormalizeImage(object):
    """normalize image
    Args:
        mean (list): im - mean
        std (list): im / std
        is_scale (bool): whether need im / 255
        norm_type (str): type in ['mean_std', 'none']
    """

    def __init__(self, mean, std, is_scale=True, norm_type="mean_std"):
        self.mean = mean
        self.std = std
        self.is_scale = is_scale
        self.norm_type = norm_type

    def __call__(self, im):
        """
        Args:
            im (np.ndarray): image (np.ndarray)
            im_info (dict): info of image
        Returns:
            im (np.ndarray):  processed image (np.ndarray)
            im_info (dict): info of processed image
        """
        im = im.astype(np.float32, copy=False)
        if self.is_scale:
            scale = 1.0 / 255.0
            im *= scale

        if self.norm_type == "mean_std":
            multiply_factor = 1 / np.array(self.std)
            subtract_factor = np.array(self.mean) / np.array(self.std)
            im = np.multiply(im, multiply_factor)
            im = np.subtract(im, subtract_factor)
        return im


class Permute(object):
    """permute image
    Args:
        to_bgr (bool): whether convert RGB to BGR
        channel_first (bool): whether convert HWC to CHW
    """

    def __init__(
        self,
    ):
        super(Permute, self).__init__()

    def __call__(self, im):
        """
        Args:
            im (np.ndarray): image (np.ndarray)
            im_info (dict): info of image
        Returns:
            im (np.ndarray):  processed image (np.ndarray)
            im_info (dict): info of processed image
        """
        im = im.transpose((2, 0, 1)).copy()
        return im


class Resize(object):
    """resize image by target_size and max_size
    Args:
        target_size (int): the target size of image
        keep_ratio (bool): whether keep_ratio or not, default true
        interp (int): method of resize
    """

    def __init__(self, target_size, keep_ratio=True, interp=cv2.INTER_LINEAR):
        if isinstance(target_size, int):
            target_size = [target_size, target_size]
        self.target_size = target_size
        self.keep_ratio = keep_ratio
        self.interp = interp

    def __call__(self, im):
        """
        Args:
            im (np.ndarray): image (np.ndarray)
            im_info (dict): info of image
        Returns:
            im (np.ndarray):  processed image (np.ndarray)
            im_info (dict): info of processed image
        """
        assert len(self.target_size) == 2
        assert self.target_size[0] > 0 and self.target_size[1] > 0
        im_scale_y, im_scale_x = self.generate_scale(im)
        im = cv2.resize(
            im, None, None, fx=im_scale_x, fy=im_scale_y, interpolation=self.interp
        )
        return im

    def generate_scale(self, im):
        """generat scale for resize image """
        origin_shape = im.shape[:2]
        if self.keep_ratio:
            im_size_min = np.min(origin_shape)
            im_size_max = np.max(origin_shape)
            target_size_min = np.min(self.target_size)
            target_size_max = np.max(self.target_size)
            im_scale = float(target_size_min) / float(im_size_min)
            if np.round(im_scale * im_size_max) > target_size_max:
                im_scale = float(target_size_max) / float(im_size_max)
            im_scale_x = im_scale
            im_scale_y = im_scale
        else:
            resize_h, resize_w = self.target_size
            im_scale_y = resize_h / float(origin_shape[0])
            im_scale_x = resize_w / float(origin_shape[1])
        return im_scale_y, im_scale_x


class Preprocess(object):
    """Preprocess interface"""
    
    def __init__(self, infer_yml):
        """init"""
        self.preprocess_ops = []
        for op_info in infer_yml["Preprocess"]:
            new_op_info = op_info.copy()
            op_type = new_op_info.pop("type")
            self.preprocess_ops.append(eval(op_type)(**new_op_info))

    def __call__(self, im):
        """entry"""
        im = decode_image(im)
        size = im.shape[0:2]
        for operator in self.preprocess_ops:
            im = operator(im)
        im = im.astype("float32")
        return im, size