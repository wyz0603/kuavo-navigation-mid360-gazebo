#!/usr/bin/env python3
"""
Shared helpers for Qwen VL ROS integration tests.
"""

import os
import tempfile
import threading
from typing import Optional

import rospy

from create_test_image import create_test_image
from qwen_vl.msg import QwenVLResponse

DEFAULT_IMAGE_PATH = os.path.join(tempfile.gettempdir(), "qwen_vl_test_image.png")
DEFAULT_TIMEOUT = 60.0


def ensure_test_image(image_path: str = DEFAULT_IMAGE_PATH) -> str:
    """
    Create a deterministic test image if it does not already exist.

    Returns:
        The path to the generated (or existing) test image.
    """
    os.makedirs(os.path.dirname(image_path), exist_ok=True)

    if not os.path.exists(image_path):
        create_test_image(image_path)

    return image_path


class ResponseWaiter:
    """Waits for a QwenVLResponse message with the expected UUID."""

    def __init__(self, expected_uuid: str):
        self._expected_uuid = expected_uuid
        self._event = threading.Event()
        self._message: Optional[QwenVLResponse] = None
        self._subscriber = rospy.Subscriber(
            "/qwen_vl_response", QwenVLResponse, self._callback
        )

    def _callback(self, msg: QwenVLResponse) -> None:
        if msg.uuid == self._expected_uuid:
            self._message = msg
            self._event.set()

    def wait(self, timeout: float) -> Optional[QwenVLResponse]:
        """Block until the expected message arrives or timeout expires."""
        received = self._event.wait(timeout=timeout)
        self._subscriber.unregister()
        return self._message if received else None
