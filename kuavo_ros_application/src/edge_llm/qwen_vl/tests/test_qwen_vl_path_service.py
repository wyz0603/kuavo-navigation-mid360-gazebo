#!/usr/bin/env python3
"""
Integration test for the /qwen_vl_query_path ROS service.

This script:
 1. Ensures a test image exists (creates one if necessary).
 2. Calls the filesystem path service with a sample question.
 3. Waits for the /qwen_vl_response topic to emit the matching UUID.
 4. Reports success/failure via the process exit code.

Usage (within a sourced ROS environment while the qwen_vl node is running):
    python3 test_qwen_vl_path_service.py [--image IMAGE_PATH] [--timeout SECONDS]
"""

import argparse
import sys

import rospy
from qwen_vl.srv import QwenVLQueryPath, QwenVLQueryPathRequest

from qwen_vl_test_utils import (
    DEFAULT_IMAGE_PATH,
    DEFAULT_TIMEOUT,
    ResponseWaiter,
    ensure_test_image,
)
DEFAULT_QUERY = "请描述图片中的图形和颜色。"


def run_test(image_path: str, query: str, timeout: float) -> int:
    """Execute the filesystem path service test."""
    rospy.loginfo("Preparing test image at %s", image_path)
    ensure_test_image(image_path)

    rospy.loginfo("Waiting for /qwen_vl_query_path service...")
    rospy.wait_for_service("/qwen_vl_query_path", timeout=timeout)

    service = rospy.ServiceProxy("/qwen_vl_query_path", QwenVLQueryPath)
    request = QwenVLQueryPathRequest()
    request.query = query
    request.image_path = image_path

    rospy.loginfo("Calling /qwen_vl_query_path with query: %s", query)
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
    print("=== PATH SERVICE TEST PASSED ===")
    print(f"UUID: {message.uuid}")
    print(f"Query: {message.query}")
    print(f"Image Info: {message.image_info}")
    print(f"Response: {message.response}")
    return 0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Test /qwen_vl_query_path service using a generated image."
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
    rospy.init_node("test_qwen_vl_path_service", anonymous=True)

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
