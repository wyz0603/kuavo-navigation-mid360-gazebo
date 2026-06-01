#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Qwen Vision-Language (VL) ROS Node

This node provides ROS services for querying the Qwen VL API with both text and images,
and publishes responses via a ROS topic. It supports two service types:
1. Base64-encoded images
2. Filesystem path to images

Services:
    /qwen_vl_query_base64 (qwen_vl/QwenVLQueryBase64)
        - Input: query string, image_base64 (base64-encoded image)
        - Output: success, error_code, message, uuid

    /qwen_vl_query_path (qwen_vl/QwenVLQueryPath)
        - Input: query string, image_path (filesystem path)
        - Output: success, error_code, message, uuid

Topic: /qwen_vl_response (qwen_vl/QwenVLResponse)
    - Publishes VL model responses with uuid, query, image_info, response, and timestamp

Error Codes:
    - 0: SUCCESS - Query accepted and processing
    - 1: EMPTY_QUERY - Query string is empty or whitespace only
    - 2: SERVICE_UNAVAILABLE - VL API service on port 9000 is not reachable
    - 3: INVALID_IMAGE - Image data is invalid or cannot be processed
"""

import rospy
import requests
import json
import hashlib
import time
import threading
import base64
import os
from io import BytesIO
from PIL import Image

from qwen_vl.srv import QwenVLQueryBase64, QwenVLQueryBase64Response
from qwen_vl.srv import QwenVLQueryPath, QwenVLQueryPathResponse
from qwen_vl.msg import QwenVLResponse


# Error code constants
ERROR_SUCCESS = 0
ERROR_EMPTY_QUERY = 1
ERROR_SERVICE_UNAVAILABLE = 2
ERROR_INVALID_IMAGE = 3


class QwenVLNode:
    """ROS node for Qwen Vision-Language model service"""

    def __init__(self):
        # Initialize ROS node
        rospy.init_node('qwen_vl_node', anonymous=True)

        # Configuration
        self.api_url = rospy.get_param('~api_url', 'http://localhost:9000/v1/chat/completions')
        self.model = rospy.get_param('~model', 'Qwen2.5-VL-7B-Instruct')
        self.temperature = rospy.get_param('~temperature', 0.7)
        self.max_tokens = rospy.get_param('~max_tokens', 300)

        # Create response publisher
        self.response_pub = rospy.Publisher('/qwen_vl_response', QwenVLResponse, queue_size=10)

        # Create services
        self.service_base64 = rospy.Service('/qwen_vl_query_base64', QwenVLQueryBase64,
                                           self.handle_query_base64)
        self.service_path = rospy.Service('/qwen_vl_query_path', QwenVLQueryPath,
                                         self.handle_query_path)

        rospy.loginfo("Qwen VL Node started")
        rospy.loginfo(f"API URL: {self.api_url}")
        rospy.loginfo(f"Model: {self.model}")
        rospy.loginfo("Service '/qwen_vl_query_base64' ready (accepts base64 images)")
        rospy.loginfo("Service '/qwen_vl_query_path' ready (accepts filesystem paths)")
        rospy.loginfo("Publishing responses to '/qwen_vl_response'")

    def generate_uuid(self, query, image_info):
        """
        Generate a UUID based on query content, image info, and timestamp

        Args:
            query (str): The query string
            image_info (str): Image information (path or base64 prefix)

        Returns:
            str: UUID string
        """
        # Combine query, image info, and current timestamp
        timestamp = str(time.time())
        content = f"{query}_{image_info[:50]}_{timestamp}"

        # Generate MD5 hash as UUID
        uuid = hashlib.md5(content.encode('utf-8')).hexdigest()

        return uuid

    def check_service_availability(self):
        """
        Check if the VL API service is available

        Returns:
            bool: True if service is available, False otherwise
        """
        try:
            # Try to connect to the service with a short timeout
            base_url = self.api_url.rsplit('/', 1)[0]  # Get base URL without endpoint
            response = requests.get(base_url, timeout=2)
            return True
        except (requests.exceptions.RequestException, requests.exceptions.Timeout):
            return False

    def validate_image_base64(self, image_base64):
        """
        Validate and normalize base64 image data

        Args:
            image_base64 (str): Base64-encoded image (with or without data URI prefix)
                               Empty string is valid for text-only mode

        Returns:
            tuple: (bool, str) - (is_valid, normalized_base64_with_prefix or empty string)
        """
        # Empty image is valid for text-only mode
        if not image_base64 or image_base64.strip() == "":
            return True, ""

        try:
            # Remove data URI prefix if present
            if image_base64.startswith('data:image'):
                # Already has prefix, validate it can be decoded
                parts = image_base64.split(',', 1)
                if len(parts) != 2:
                    return False, ""
                base64_data = parts[1]
            else:
                base64_data = image_base64

            # Try to decode and verify it's a valid image
            image_bytes = base64.b64decode(base64_data)
            img = Image.open(BytesIO(image_bytes))
            img.verify()  # Verify it's a valid image

            # Return normalized format with data URI prefix
            if image_base64.startswith('data:image'):
                return True, image_base64
            else:
                # Add data URI prefix
                return True, f"data:image/png;base64,{base64_data}"

        except Exception as e:
            rospy.logerr(f"Invalid base64 image data: {e}")
            return False, ""

    def load_image_from_path(self, image_path):
        """
        Load image from filesystem and convert to base64 data URI

        Args:
            image_path (str): Path to image file
                             Empty string is valid for text-only mode

        Returns:
            tuple: (bool, str) - (success, base64_data_uri or empty string)
        """
        # Empty path is valid for text-only mode
        if not image_path or image_path.strip() == "":
            return True, ""

        try:
            if not os.path.exists(image_path):
                rospy.logerr(f"Image file does not exist: {image_path}")
                return False, ""

            # Load image
            img = Image.open(image_path)

            # Convert to base64
            buffered = BytesIO()
            img_format = img.format if img.format else 'PNG'
            img.save(buffered, format=img_format)
            img_base64 = base64.b64encode(buffered.getvalue()).decode()

            # Create data URI
            mime_type = f"image/{img_format.lower()}"
            data_uri = f"data:{mime_type};base64,{img_base64}"

            return True, data_uri

        except Exception as e:
            rospy.logerr(f"Error loading image from path {image_path}: {e}")
            return False, ""

    def _normalize_response_text(self, response_content):
        """
        Convert VL API response content into a UTF-8 safe string.

        Args:
            response_content: Raw content structure from the VL API response.

        Returns:
            str: UTF-8 encoded string suitable for ROS topic publishing.
        """

        def _extract_text(item):
            if isinstance(item, dict):
                # Handle common multimodal response formats
                if isinstance(item.get('text'), str):
                    return item.get('text')
                if item.get('type') == 'image_url':
                    image_url = item.get('image_url', {}).get('url')
                    if isinstance(image_url, str):
                        return f"[image] {image_url}"
                if 'content' in item:
                    return _extract_text(item['content'])
                return json.dumps(item, ensure_ascii=False)

            if isinstance(item, list):
                parts = [_extract_text(sub_item) for sub_item in item]
                return "\n".join(part for part in parts if part)

            if item is None:
                return ""

            if isinstance(item, bytes):
                return item.decode('utf-8', errors='replace')

            return str(item)

        if isinstance(response_content, list):
            normalized = "\n".join(
                part for part in (_extract_text(element) for element in response_content) if part
            )
        else:
            normalized = _extract_text(response_content)

        if isinstance(normalized, bytes):
            normalized = normalized.decode('utf-8', errors='replace')
        else:
            normalized = normalized.encode('utf-8', errors='replace').decode('utf-8')

        return normalized.strip()

    def query_vl_api(self, query, image_base64_uri, uuid, image_info):
        """
        Query the VL API and publish response to topic

        This runs in a separate thread to avoid blocking the service response.
        Supports both text-only mode (when image_base64_uri is empty) and
        vision-language mode (when image is provided).

        Args:
            query (str): The query string
            image_base64_uri (str): Base64 data URI of the image (empty for text-only mode)
            uuid (str): The UUID for this query
            image_info (str): Information about the image (for logging)
        """
        try:
            # Prepare request payload based on whether image is provided
            if image_base64_uri:
                # Vision-language mode: include image
                content = [
                    {"type": "image_url", "image_url": {"url": image_base64_uri}},
                    {"type": "text", "text": query}
                ]
            else:
                # Text-only mode: just send the query string
                content = query

            payload = {
                "messages": [
                    {
                        "role": "user",
                        "content": content
                    }
                ],
                "max_tokens": self.max_tokens
            }

            rospy.loginfo(f"Sending VL query to API: {query[:50]}... (UUID: {uuid})")

            # Send HTTP POST request
            response = requests.post(
                self.api_url,
                headers={"Content-Type": "application/json"},
                json=payload,
                timeout=60  # VL models may take longer
            )

            # Check response status
            response.raise_for_status()

            # Parse response
            response_data = response.json()

            # Extract the assistant's message from response and normalize
            vl_response = "Error: No response from VL model"
            if isinstance(response_data, dict) and response_data.get('choices'):
                first_choice = response_data['choices'][0]
                message_payload = first_choice.get('message', {})
                raw_content = message_payload.get('content')
                normalized = self._normalize_response_text(raw_content)
                if normalized:
                    vl_response = normalized
                else:
                    rospy.logwarn(f"Empty content in VL response: {response_data}")
            else:
                rospy.logerr(f"Invalid response format: {response_data}")

            # Create and publish ROS message
            msg = QwenVLResponse()
            msg.uuid = uuid
            msg.query = self._normalize_response_text(query)
            msg.image_info = self._normalize_response_text(image_info)
            msg.response = vl_response
            msg.timestamp = rospy.Time.now()

            self.response_pub.publish(msg)
            rospy.loginfo(f"Published VL response for UUID: {uuid}")

        except requests.exceptions.Timeout:
            rospy.logerr(f"Timeout querying VL API for UUID: {uuid}")
            self._publish_error_response(uuid, query, image_info, "Request timeout")

        except requests.exceptions.RequestException as e:
            rospy.logerr(f"Error querying VL API for UUID {uuid}: {e}")
            self._publish_error_response(uuid, query, image_info, f"HTTP error: {str(e)}")

        except Exception as e:
            rospy.logerr(f"Unexpected error for UUID {uuid}: {e}")
            self._publish_error_response(uuid, query, image_info, f"Unexpected error: {str(e)}")

    def _publish_error_response(self, uuid, query, image_info, error_message):
        """Publish an error response to the topic"""
        msg = QwenVLResponse()
        msg.uuid = uuid
        msg.query = self._normalize_response_text(query)
        msg.image_info = self._normalize_response_text(image_info)
        msg.response = self._normalize_response_text(f"Error: {error_message}")
        msg.timestamp = rospy.Time.now()
        self.response_pub.publish(msg)

    def handle_query_base64(self, req):
        """
        Handle incoming base64 image service requests

        Supports both vision-language mode (with image) and text-only mode (empty image).

        Args:
            req: QwenVLQueryBase64Request with query and image_base64 fields

        Returns:
            QwenVLQueryBase64Response with success, error_code, message, and uuid
        """
        query = req.query
        image_base64 = req.image_base64
        response = QwenVLQueryBase64Response()

        # Validate query is not empty
        if not query or query.strip() == "":
            rospy.logwarn("Received empty query request")
            response.success = False
            response.error_code = ERROR_EMPTY_QUERY
            response.message = "Query string is empty"
            response.uuid = ""
            return response

        # Validate image base64 (empty is valid for text-only mode)
        is_valid, normalized_image = self.validate_image_base64(image_base64)
        if not is_valid:
            rospy.logwarn("Received invalid base64 image data")
            response.success = False
            response.error_code = ERROR_INVALID_IMAGE
            response.message = "Invalid base64 image data"
            response.uuid = ""
            return response

        # Check if API service is available
        if not self.check_service_availability():
            rospy.logerr(f"VL API service at {self.api_url} is not available")
            response.success = False
            response.error_code = ERROR_SERVICE_UNAVAILABLE
            response.message = f"VL API service at port 9000 is not available"
            response.uuid = ""
            return response

        # Generate UUID for this query
        image_mode = "text_only" if not normalized_image else "base64_image"
        uuid = self.generate_uuid(query, image_mode)

        if normalized_image:
            rospy.loginfo(f"Received base64 VL query (UUID: {uuid}): {query[:50]}...")
        else:
            rospy.loginfo(f"Received text-only query (UUID: {uuid}): {query[:50]}...")

        # Start API query in a separate thread (non-blocking)
        thread = threading.Thread(
            target=self.query_vl_api,
            args=(query, normalized_image, uuid, image_mode)
        )
        thread.daemon = True
        thread.start()

        # Return success with UUID
        response.success = True
        response.error_code = ERROR_SUCCESS
        response.message = "Query accepted"
        response.uuid = uuid

        return response

    def handle_query_path(self, req):
        """
        Handle incoming filesystem path image service requests

        Supports both vision-language mode (with image) and text-only mode (empty path).

        Args:
            req: QwenVLQueryPathRequest with query and image_path fields

        Returns:
            QwenVLQueryPathResponse with success, error_code, message, and uuid
        """
        query = req.query
        image_path = req.image_path
        response = QwenVLQueryPathResponse()

        # Validate query is not empty
        if not query or query.strip() == "":
            rospy.logwarn("Received empty query request")
            response.success = False
            response.error_code = ERROR_EMPTY_QUERY
            response.message = "Query string is empty"
            response.uuid = ""
            return response

        # Load image from path (empty path is valid for text-only mode)
        success, image_base64_uri = self.load_image_from_path(image_path)
        if not success:
            rospy.logwarn(f"Failed to load image from path: {image_path}")
            response.success = False
            response.error_code = ERROR_INVALID_IMAGE
            response.message = f"Cannot load image from path: {image_path}"
            response.uuid = ""
            return response

        # Check if API service is available
        if not self.check_service_availability():
            rospy.logerr(f"VL API service at {self.api_url} is not available")
            response.success = False
            response.error_code = ERROR_SERVICE_UNAVAILABLE
            response.message = f"VL API service at port 9000 is not available"
            response.uuid = ""
            return response

        # Generate UUID for this query
        image_mode = "text_only" if not image_base64_uri else f"path:{image_path}"
        uuid = self.generate_uuid(query, image_mode)

        if image_base64_uri:
            rospy.loginfo(f"Received path VL query (UUID: {uuid}): {query[:50]}... [image: {image_path}]")
        else:
            rospy.loginfo(f"Received text-only query (UUID: {uuid}): {query[:50]}...")

        # Start API query in a separate thread (non-blocking)
        thread = threading.Thread(
            target=self.query_vl_api,
            args=(query, image_base64_uri, uuid, image_mode)
        )
        thread.daemon = True
        thread.start()

        # Return success with UUID
        response.success = True
        response.error_code = ERROR_SUCCESS
        response.message = "Query accepted"
        response.uuid = uuid

        return response

    def run(self):
        """Keep the node running"""
        rospy.spin()


if __name__ == '__main__':
    try:
        node = QwenVLNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
