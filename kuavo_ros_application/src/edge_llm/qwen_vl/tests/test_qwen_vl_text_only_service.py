#!/usr/bin/env python3
"""
Integration test for text-only mode via ROS services.

This script tests that the Qwen VL ROS services can handle text-only queries
(with empty image parameter) and fall back to pure text dialogue mode.

Tests both service interfaces:
 1. /qwen_vl_query_base64 with empty image_base64
 2. /qwen_vl_query_path with empty image_path

Usage:
    python3 test_qwen_vl_text_only_service.py [--timeout SECONDS]
"""

import argparse
import sys

import rospy
from qwen_vl.srv import QwenVLQueryBase64, QwenVLQueryBase64Request
from qwen_vl.srv import QwenVLQueryPath, QwenVLQueryPathRequest

from qwen_vl_test_utils import (
    DEFAULT_TIMEOUT,
    ResponseWaiter,
)

DEFAULT_QUERY = "什么是大型语言模型？请简要说明。"


def test_base64_text_only(query: str, timeout: float) -> int:
    """Test text-only mode via /qwen_vl_query_base64 service."""
    rospy.loginfo("=== Testing text-only mode via base64 service ===")

    rospy.loginfo("Waiting for /qwen_vl_query_base64 service...")
    rospy.wait_for_service("/qwen_vl_query_base64", timeout=timeout)

    service = rospy.ServiceProxy("/qwen_vl_query_base64", QwenVLQueryBase64)
    request = QwenVLQueryBase64Request()
    request.query = query
    request.image_base64 = ""  # Empty image for text-only mode

    rospy.loginfo("Calling /qwen_vl_query_base64 with text-only query: %s", query)
    response = service(request)

    if not response.success:
        rospy.logerr(
            "Service returned failure. code=%s message=%s",
            response.error_code,
            response.message,
        )
        return 1

    rospy.loginfo("Service accepted text-only query. UUID=%s", response.uuid)
    waiter = ResponseWaiter(response.uuid)
    message = waiter.wait(timeout=timeout)

    if message is None:
        rospy.logerr("Timed out waiting for /qwen_vl_response with UUID %s", response.uuid)
        return 1

    rospy.loginfo("Received response: %s", message.response)
    print("=== BASE64 TEXT-ONLY SERVICE TEST PASSED ===")
    print(f"UUID: {message.uuid}")
    print(f"Query: {message.query}")
    print(f"Image Info: {message.image_info}")
    print(f"Response: {message.response}")
    return 0


def test_path_text_only(query: str, timeout: float) -> int:
    """Test text-only mode via /qwen_vl_query_path service."""
    rospy.loginfo("=== Testing text-only mode via path service ===")

    rospy.loginfo("Waiting for /qwen_vl_query_path service...")
    rospy.wait_for_service("/qwen_vl_query_path", timeout=timeout)

    service = rospy.ServiceProxy("/qwen_vl_query_path", QwenVLQueryPath)
    request = QwenVLQueryPathRequest()
    request.query = query
    request.image_path = ""  # Empty path for text-only mode

    rospy.loginfo("Calling /qwen_vl_query_path with text-only query: %s", query)
    response = service(request)

    if not response.success:
        rospy.logerr(
            "Service returned failure. code=%s message=%s",
            response.error_code,
            response.message,
        )
        return 1

    rospy.loginfo("Service accepted text-only query. UUID=%s", response.uuid)
    waiter = ResponseWaiter(response.uuid)
    message = waiter.wait(timeout=timeout)

    if message is None:
        rospy.logerr("Timed out waiting for /qwen_vl_response with UUID %s", response.uuid)
        return 1

    rospy.loginfo("Received response: %s", message.response)
    print("=== PATH TEXT-ONLY SERVICE TEST PASSED ===")
    print(f"UUID: {message.uuid}")
    print(f"Query: {message.query}")
    print(f"Image Info: {message.image_info}")
    print(f"Response: {message.response}")
    return 0


def run_test(query: str, timeout: float) -> int:
    """Execute both text-only service tests."""
    rospy.loginfo("Starting text-only service tests")

    # Test 1: base64 service with empty image
    result = test_base64_text_only(query, timeout)
    if result != 0:
        return result

    print()  # Separator between tests

    # Test 2: path service with empty path
    result = test_path_text_only(query, timeout)
    if result != 0:
        return result

    print("\n=== ALL TEXT-ONLY SERVICE TESTS PASSED ===")
    return 0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Test text-only mode via Qwen VL ROS services."
    )
    parser.add_argument(
        "--query",
        default=DEFAULT_QUERY,
        help="Question to send in text-only mode.",
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
    rospy.init_node("test_qwen_vl_text_only_service", anonymous=True)

    try:
        return run_test(args.query, args.timeout)
    except rospy.ROSException as exc:
        rospy.logerr("ROS exception: %s", exc)
        return 1
    except Exception as exc:  # pylint: disable=broad-except
        rospy.logerr("Unexpected error: %s", exc)
        return 1


if __name__ == "__main__":
    sys.exit(main())
