#!/usr/bin/env python3
"""
Convert an image file to base64 encoding.
This script is used for testing the Qwen VL ROS node's base64 image service.
"""

import base64
import sys


def convert_image_to_base64(image_path):
    """
    Convert an image file to base64 string.

    Args:
        image_path: Path to the image file

    Returns:
        Base64-encoded string of the image
    """
    with open(image_path, 'rb') as f:
        img_data = f.read()
        b64 = base64.b64encode(img_data).decode()
        return b64


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: convert_image_to_base64.py <image_path>")
        sys.exit(1)

    image_path = sys.argv[1]
    b64_string = convert_image_to_base64(image_path)
    print(b64_string)
