#!/usr/bin/env python3
"""Test script for the Qwen ROS node service and response topic.

This script replaces the previous shell-based workflow by implementing the
entire flow in Python so we can reuse the existing ROS Python APIs directly.
"""

import argparse
import sys
import threading
import rosgraph
import rospy

from typing import Optional

from qwen_txt.msg import QwenResponse
from qwen_txt.srv import QwenQuery, QwenQueryResponse

GREEN = "\033[0;32m"
YELLOW = "\033[1;33m"
RED = "\033[0;31m"
NC = "\033[0m"

DEFAULT_QUERY = "你好，请介绍一下你自己"
SERVICE_NAME = "/qwen_query"
RESPONSE_TOPIC = "/qwen_response"
TIMEOUT_SECONDS = 60.0


def exit_with_failure(message: str, exit_code: int = 1) -> None:
    """Print a failure message and terminate with the provided exit code."""

    print(f"{RED}{message}{NC}")
    print(f"{YELLOW}Exiting with status {exit_code}.{NC}")
    sys.exit(exit_code)


class ResponseWaiter:
    """Utility to wait for a QwenResponse message matching a UUID."""

    def __init__(self) -> None:
        self._target_uuid: Optional[str] = None
        self._event = threading.Event()
        self._message: Optional[QwenResponse] = None
        self._lock = threading.Lock()

    def set_target(self, uuid: str) -> None:
        with self._lock:
            self._target_uuid = uuid
            self._event.clear()
            self._message = None

    def callback(self, msg: QwenResponse) -> None:
        with self._lock:
            if self._target_uuid and msg.uuid == self._target_uuid:
                self._message = msg
                self._event.set()

    def wait(self, timeout: float) -> Optional[QwenResponse]:
        if not self._event.wait(timeout):
            return None
        with self._lock:
            return self._message


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Call the Qwen query service and display the response"
    )
    parser.add_argument(
        "query",
        nargs=argparse.REMAINDER,
        help="Query text to send to /qwen_query (default in Chinese if omitted)",
    )
    return parser.parse_args()


def build_query(args: argparse.Namespace) -> str:
    if args.query:
        return " ".join(args.query)
    return DEFAULT_QUERY


def ensure_ros_master() -> None:
    if rosgraph.is_master_online():
        return
    print(f"{RED}Error: roscore is not running!{NC}")
    print("Please start roscore first: roscore &")
    exit_with_failure("ROS master not detected.")


def wait_for_service() -> None:
    print(f"{YELLOW}Checking if {SERVICE_NAME} service is available...{NC}")
    try:
        rospy.wait_for_service(SERVICE_NAME, timeout=10.0)
    except rospy.ROSException:
        print(f"{RED}Error: {SERVICE_NAME} service not found!{NC}")
        print("Please start the Qwen node first: rosrun qwen_txt main.py")
        exit_with_failure("Service unavailable.")


def call_service(query: str) -> QwenQueryResponse:
    service = rospy.ServiceProxy(SERVICE_NAME, QwenQuery)
    print(f"{YELLOW}Calling {SERVICE_NAME} service...{NC}")
    try:
        response = service(query=query)
    except rospy.ServiceException as exc:
        exit_with_failure(f"Error calling service: {exc}")

    status = "success" if response.success else "failure"
    print(
        f"Service call completed with status={status}, "
        f"error_code={response.error_code}, message='{response.message}'"
    )

    if not response.success:
        exit_with_failure("Service reported failure.")

    return response


def wait_for_response(uuid: str, waiter: ResponseWaiter) -> QwenResponse:
    print(f"{YELLOW}Waiting for response on {RESPONSE_TOPIC} topic...{NC}")
    waiter.set_target(uuid)
    message = waiter.wait(TIMEOUT_SECONDS)
    if message is None:
        exit_with_failure(
            f"Timeout: No response received within {TIMEOUT_SECONDS} seconds."
        )

    print(f"{GREEN}Response received successfully!{NC}")
    print("=" * 50)
    print(f"UUID: {message.uuid}")
    print(f"Query: {message.query}")
    print(f"Response: {message.response}")
    print(f"Timestamp: {message.timestamp.to_sec()}")
    print("=" * 50)
    return message


def main() -> None:
    args = parse_args()
    query = build_query(args)

    print(f"{GREEN}========================================")
    print("Qwen ROS Node Test Script")
    print(f"========================================{NC}")
    print("")
    print(f"{YELLOW}Query:{NC} {query}")
    print("")

    ensure_ros_master()

    rospy.init_node("qwen_test_runner", anonymous=True, disable_signals=True)

    wait_for_service()

    waiter = ResponseWaiter()
    rospy.Subscriber(RESPONSE_TOPIC, QwenResponse, waiter.callback)

    response = call_service(query)

    print(f"Received UUID: {YELLOW}{response.uuid}{NC}")

    wait_for_response(response.uuid, waiter)
    print(f"{GREEN}Service + topic validation succeeded; exiting with status 0.{NC}")
    sys.exit(0)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nOperation interrupted by user.")
        sys.exit(1)
    except rospy.ROSInterruptException:
        print("\nROS shutdown detected before completion.")
        sys.exit(1)
