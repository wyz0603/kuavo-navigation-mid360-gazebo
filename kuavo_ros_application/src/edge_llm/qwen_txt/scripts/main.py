#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Qwen Text LLM ROS Node

This node provides a ROS service for querying the Qwen LLM API and publishes
responses via a ROS topic. It acts as a bridge between ROS and the HTTP API
running on port 9000.

Service: /qwen_query (qwen_txt/QwenQuery)
    - Input: query string
    - Output:
        - success (bool): Service call success status
        - error_code (int32): Error code (0=success, 1=empty query, 2=service unavailable)
        - message (string): Status or error message
        - uuid (string): UUID for tracking the request (empty if failed)

Topic: /qwen_response (qwen_txt/QwenResponse)
    - Publishes LLM responses with uuid, query, response, and timestamp

Error Codes:
    - 0: SUCCESS - Query accepted and processing
    - 1: EMPTY_QUERY - Query string is empty or whitespace only
    - 2: SERVICE_UNAVAILABLE - LLM API service on port 9000 is not reachable
"""

import rospy
import requests
import json
import hashlib
import time
import threading
from qwen_txt.srv import QwenQuery, QwenQueryResponse
from qwen_txt.msg import QwenResponse


# Error code constants
ERROR_SUCCESS = 0
ERROR_EMPTY_QUERY = 1
ERROR_SERVICE_UNAVAILABLE = 2


class QwenTextNode:
    """ROS node for Qwen text LLM service"""

    def __init__(self):
        # Initialize ROS node
        rospy.init_node('qwen_text_node', anonymous=True)

        # Configuration
        self.api_url = rospy.get_param('~api_url', 'http://localhost:9000/v1/chat/completions')
        self.model = rospy.get_param('~model', 'Qwen2.5-7B-Instruct-q4f16_ft-MLC')
        self.temperature = rospy.get_param('~temperature', 0.7)
        self.max_tokens = rospy.get_param('~max_tokens', 500)

        # Create response publisher
        self.response_pub = rospy.Publisher('/qwen_response', QwenResponse, queue_size=10)

        # Create service
        self.service = rospy.Service('/qwen_query', QwenQuery, self.handle_query)

        rospy.loginfo("Qwen Text Node started")
        rospy.loginfo(f"API URL: {self.api_url}")
        rospy.loginfo(f"Model: {self.model}")
        rospy.loginfo("Service '/qwen_query' ready")
        rospy.loginfo("Publishing responses to '/qwen_response'")

    def generate_uuid(self, query):
        """
        Generate a UUID based on query content and timestamp

        Args:
            query (str): The query string

        Returns:
            str: UUID string
        """
        # Combine query with current timestamp (microseconds for uniqueness)
        timestamp = str(time.time())
        content = f"{query}_{timestamp}"

        # Generate MD5 hash as UUID
        uuid = hashlib.md5(content.encode('utf-8')).hexdigest()

        return uuid

    def check_service_availability(self):
        """
        Check if the LLM API service is available

        Returns:
            bool: True if service is available, False otherwise
        """
        try:
            # Try to connect to the service with a short timeout
            response = requests.get(
                self.api_url.rsplit('/', 1)[0],  # Get base URL without endpoint
                timeout=2
            )
            return True
        except (requests.exceptions.RequestException, requests.exceptions.Timeout):
            return False

    def query_llm_api(self, query, uuid):
        """
        Query the LLM API and publish response to topic

        This runs in a separate thread to avoid blocking the service response.

        Args:
            query (str): The query string
            uuid (str): The UUID for this query
        """
        try:
            # Prepare request payload
            payload = {
                "model": self.model,
                "messages": [
                    {
                        "role": "user",
                        "content": query
                    }
                ],
                "temperature": self.temperature,
                "max_tokens": self.max_tokens
            }

            rospy.loginfo(f"Sending query to LLM API: {query[:50]}... (UUID: {uuid})")

            # Send HTTP POST request
            response = requests.post(
                self.api_url,
                headers={"Content-Type": "application/json"},
                json=payload,
                timeout=30
            )

            # Check response status
            response.raise_for_status()

            # Parse response
            response_data = response.json()

            # Extract the assistant's message from response
            if 'choices' in response_data and len(response_data['choices']) > 0:
                llm_response = response_data['choices'][0]['message']['content']
            else:
                llm_response = "Error: No response from LLM"
                rospy.logerr(f"Invalid response format: {response_data}")

            # Create and publish ROS message
            msg = QwenResponse()
            msg.uuid = uuid
            msg.query = query
            msg.response = llm_response
            msg.timestamp = rospy.Time.now()

            self.response_pub.publish(msg)
            rospy.loginfo(f"Published response for UUID: {uuid}")

        except requests.exceptions.Timeout:
            rospy.logerr(f"Timeout querying LLM API for UUID: {uuid}")
            self._publish_error_response(uuid, query, "Request timeout")

        except requests.exceptions.RequestException as e:
            rospy.logerr(f"Error querying LLM API for UUID {uuid}: {e}")
            self._publish_error_response(uuid, query, f"HTTP error: {str(e)}")

        except Exception as e:
            rospy.logerr(f"Unexpected error for UUID {uuid}: {e}")
            self._publish_error_response(uuid, query, f"Unexpected error: {str(e)}")

    def _publish_error_response(self, uuid, query, error_message):
        """Publish an error response to the topic"""
        msg = QwenResponse()
        msg.uuid = uuid
        msg.query = query
        msg.response = f"Error: {error_message}"
        msg.timestamp = rospy.Time.now()
        self.response_pub.publish(msg)

    def handle_query(self, req):
        """
        Handle incoming service requests

        Args:
            req: QwenQueryRequest with query field

        Returns:
            QwenQueryResponse with success, error_code, message, and uuid fields
        """
        query = req.query
        response = QwenQueryResponse()

        # Validate query is not empty
        if not query or query.strip() == "":
            rospy.logwarn("Received empty query request")
            response.success = False
            response.error_code = ERROR_EMPTY_QUERY
            response.message = "Query string is empty"
            response.uuid = ""
            return response

        # Check if API service is available
        if not self.check_service_availability():
            rospy.logerr(f"LLM API service at {self.api_url} is not available")
            response.success = False
            response.error_code = ERROR_SERVICE_UNAVAILABLE
            response.message = f"LLM API service at port 9000 is not available"
            response.uuid = ""
            return response

        # Generate UUID for this query
        uuid = self.generate_uuid(query)

        rospy.loginfo(f"Received query request (UUID: {uuid}): {query[:50]}...")

        # Start API query in a separate thread (non-blocking)
        thread = threading.Thread(target=self.query_llm_api, args=(query, uuid))
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
        node = QwenTextNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
