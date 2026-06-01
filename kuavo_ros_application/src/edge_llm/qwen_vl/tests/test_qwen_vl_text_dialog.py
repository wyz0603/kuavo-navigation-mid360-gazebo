#!/usr/bin/env python3
"""
Smoke test for text-only dialogue with the Qwen VL HTTP API.

The ROS node relies on this backend, so exercising it directly provides
coverage for pure conversational use cases.

Usage:
    python3 test_qwen_vl_text_dialog.py [--api-url URL] [--timeout SECONDS]
"""

import argparse
import sys
from typing import List
from urllib.parse import urlsplit, urlunsplit

import requests

DEFAULT_API_URL = "http://localhost:9000/v1/chat/completions"
DEFAULT_TIMEOUT = 30.0
TEST_QUESTIONS: List[str] = [
    "什么是大型语言模型？请简要说明。",
    "用两句话介绍一下贝叶斯定理。",
    "请给出一个使用Python读取文件的示例。",
]


def call_chat_completion(api_url: str, question: str, timeout: float) -> str:
    """Send a chat completion request and return the assistant reply."""
    response = requests.post(
        api_url,
        json={
            "messages": [
                {
                    "role": "user",
                    "content": question,
                }
            ],
            "max_tokens": 256,
        },
        timeout=timeout,
    )
    response.raise_for_status()
    payload = response.json()

    if "choices" not in payload or not payload["choices"]:
        raise ValueError("Response JSON does not contain choices field")

    return payload["choices"][0]["message"]["content"]

def derive_health_url(api_url: str) -> str:
    """Return the health-check endpoint associated with the API origin."""
    parsed_url = urlsplit(api_url)
    if not parsed_url.scheme or not parsed_url.netloc:
        raise ValueError(f"API URL must include scheme and host: {api_url!r}")

    return urlunsplit((parsed_url.scheme, parsed_url.netloc, "/health", "", ""))


def run_test(api_url: str, timeout: float) -> int:
    print("=== TEXT DIALOG TEST START ===")
    print(f"API URL: {api_url}")

    try:
        health_url = derive_health_url(api_url)
        print(f"Health check URL: {health_url}")
        health_response = requests.get(health_url, timeout=timeout)
        health_response.raise_for_status()
        print(f"Health check: {health_response.json()}")
    except ValueError as exc:
        print(f"Health check failed: {exc}")
        return 1
    except requests.RequestException as exc:
        print(f"Health check failed: {exc}")
        print(
            "Hint: ensure the Qwen VL service is running and exposes /health on the same host/port "
            "as the chat completions endpoint."
        )
        return 1

    for idx, question in enumerate(TEST_QUESTIONS, start=1):
        print(f"\n问题 {idx}: {question}")
        try:
            answer = call_chat_completion(api_url, question, timeout)
        except requests.RequestException as exc:
            print(f"请求失败: {exc}")
            return 1
        except ValueError as exc:
            print(f"响应格式错误: {exc}")
            return 1

        print("回答:")
        print(answer.strip())

    print("\n=== TEXT DIALOG TEST PASSED ===")
    return 0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Test text-only dialogue via Qwen VL API.")
    parser.add_argument(
        "--api-url",
        default=DEFAULT_API_URL,
        help=f"Chat completions endpoint (default: {DEFAULT_API_URL})",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=DEFAULT_TIMEOUT,
        help="Request timeout in seconds.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        return run_test(args.api_url, args.timeout)
    except Exception as exc:  # pylint: disable=broad-except
        print(f"Unexpected error: {exc}")
        return 1


if __name__ == "__main__":
    sys.exit(main())
