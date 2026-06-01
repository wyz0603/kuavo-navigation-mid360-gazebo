#!/usr/bin/env python3
"""
Integration test for the /qwen_vl_query_base64 ROS service.

This script:
 1. Ensures a deterministic test image exists.
 2. Converts the image to base64 using the repo helper.
 3. Calls the base64 service with a sample question.
 4. Waits for the /qwen_vl_response topic to emit the matching UUID.
 5. Reports success/failure via the process exit code.

Usage:
    python3 test_qwen_vl_base64_service.py [--image IMAGE_PATH] [--timeout SECONDS]
"""

import argparse
import sys

import rospy
from qwen_vl.srv import QwenVLQueryBase64, QwenVLQueryBase64Request

from convert_image_to_base64 import convert_image_to_base64
from qwen_vl_test_utils import (
    DEFAULT_IMAGE_PATH,
    DEFAULT_TIMEOUT,
    ResponseWaiter,
    ensure_test_image,
)

DEFAULT_QUERY = "图片中有什么内容？请简要描述。"


def run_test(image_path: str, query: str, timeout: float) -> int:
    """Execute the base64 image service test."""
    rospy.loginfo("Preparing test image at %s", image_path)
    ensure_test_image(image_path)

    rospy.loginfo("Converting image to base64")
    image_base64 = convert_image_to_base64(image_path)

    rospy.loginfo("Waiting for /qwen_vl_query_base64 service...")
    rospy.wait_for_service("/qwen_vl_query_base64", timeout=timeout)

    service = rospy.ServiceProxy("/qwen_vl_query_base64", QwenVLQueryBase64)
    request = QwenVLQueryBase64Request()
    request.query = query
    request.image_base64 = image_base64

    rospy.loginfo("Calling /qwen_vl_query_base64 with query: %s", query)
    response = service(request)

    if not response.success:
        rospy.logerr(
            "Service returned failure. code=%s message=%s",
            response.error_code,
            response.message,
        )
        return 1

    rospy.loginfo("Service accepted query. UUID=%s", response.uuid)
    waiter = ResponseWaiter(response.uuid)
    message = waiter.wait(timeout=timeout)

    if message is None:
        rospy.logerr("Timed out waiting for /qwen_vl_response with UUID %s", response.uuid)
        return 1

    rospy.loginfo("Received response: %s", message.response)
    print("=== BASE64 SERVICE TEST PASSED ===")
    print(f"UUID: {message.uuid}")
    print(f"Query: {message.query}")
    print(f"Image Info: {message.image_info}")
    print(f"Response: {message.response}")
    return 0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Test /qwen_vl_query_base64 service using a generated image."
    )
    parser.add_argument(
        "--image",
        default=DEFAULT_IMAGE_PATH,
        help=f"Path to test image (default: {DEFAULT_IMAGE_PATH})",
    )
    parser.add_argument(
        "--query",
        default=DEFAULT_QUERY,
        help="Question to send with the image.",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=DEFAULT_TIMEOUT,
        help="Seconds to wait for service availability and topic response.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    rospy.init_node("test_qwen_vl_base64_service", anonymous=True)

    try:
        return run_test(args.image, args.query, args.timeout)
    except rospy.ROSException as exc:
        rospy.logerr("ROS exception: %s", exc)
        return 1
    except Exception as exc:  # pylint: disable=broad-except
        rospy.logerr("Unexpected error: %s", exc)
        return 1


if __name__ == "__main__":
    sys.exit(main())
