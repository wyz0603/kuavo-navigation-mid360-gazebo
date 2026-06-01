import numpy as np
import cv2

class Postprocess(object):
    """postprocess of picodet"""

    def __init__(self, threshold=0.5):
        self.threshold = threshold

    def filter_box(self, result, threshold):
        """filter box which's score is below threshold"""
        boxes = result["boxes"]
        boxes_num = result["boxes_num"][0]
        if boxes_num == 0:
            boxes = np.array([], dtype="float32")
        else:
            idx = boxes[:, 1] > threshold
            boxes = boxes[idx, :]
        filter_num = np.array(boxes.shape[0])
        filter_res = {"boxes": boxes, "boxes_num": filter_num}
        return filter_res

    def __call__(self, result):
        np_boxes_num = result["boxes_num"]
        assert isinstance(
            np_boxes_num, np.ndarray
        ), "`np_boxes_num` should be a `numpy.ndarray`"
        result = {k: v for k, v in result.items() if v is not None}
        result = self.filter_box(result, threshold=self.threshold)
        return result


labels = [
    "person",
    "bicycle",
    "car",
    "motorcycle",
    "airplane",
    "bus",
    "train",
    "truck",
    "boat",
    "traffic light",
    "fire hydrant",
    "stop sign",
    "parking meter",
    "bench",
    "bird",
    "cat",
    "dog",
    "horse",
    "sheep",
    "cow",
    "elephant",
    "bear",
    "zebra",
    "giraffe",
    "backpack",
    "umbrella",
    "handbag",
    "tie",
    "suitcase",
    "frisbee",
    "skis",
    "snowboard",
    "sports ball",
    "kite",
    "baseball bat",
    "baseball glove",
    "skateboard",
    "surfboard",
    "tennis racket",
    "bottle",
    "wine glass",
    "cup",
    "fork",
    "knife",
    "spoon",
    "bowl",
    "banana",
    "apple",
    "sandwich",
    "orange",
    "broccoli",
    "carrot",
    "hot dog",
    "pizza",
    "donut",
    "cake",
    "chair",
    "couch",
    "potted plant",
    "bed",
    "dining table",
    "toilet",
    "tv",
    "laptop",
    "mouse",
    "remote",
    "keyboard",
    "cell phone",
    "microwave",
    "oven",
    "toaster",
    "sink",
    "refrigerator",
    "book",
    "clock",
    "vase",
    "scissors",
    "teddy bear",
    "hair drier",
    "toothbrush",
]


def paddle_vis(img, results, threshold=0.5):
    """visualize results"""
    img = img.copy()
    for i in results["boxes"]:
        label = int(i[0])
        score = i[1]
        if score < threshold:
            continue
        xmin, ymin, xmax, ymax = i[2:]
        cv2.rectangle(
            img, (int(xmin), int(ymin)), (int(xmax), int(ymax)), (0, 255, 255)
        )
        cv2.putText(
            img,
            str(labels[label]) + " " + str(score),
            (int(xmin), int(ymin)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (255, 255, 255),
            1,
        )
    return img
