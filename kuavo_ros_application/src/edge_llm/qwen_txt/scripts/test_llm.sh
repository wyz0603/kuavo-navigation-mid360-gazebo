#!/bin/bash

# 测试 LLM API 的脚本
# 使用方法: ./test_llm.sh [问题内容]

API_URL="http://localhost:9000/v1/chat/completions"
MODEL="Qwen2.5-7B-Instruct-q4f16_ft-MLC"

# 如果提供了参数，使用参数作为问题，否则使用默认问题
if [ -z "$1" ]; then
    QUESTION="你好，请介绍一下你自己"
else
    QUESTION="$*"
fi

echo "========================================"
echo "测试问题: $QUESTION"
echo "========================================"
echo ""

# 发送请求并解析响应，使用 ensure_ascii=False 保持 UTF-8 编码
curl -s -X POST "$API_URL" \
  -H "Content-Type: application/json" \
  -d "{
    \"model\": \"$MODEL\",
    \"messages\": [
      {
        \"role\": \"user\",
        \"content\": \"$QUESTION\"
      }
    ],
    \"temperature\": 0.7,
    \"max_tokens\": 500
  }" | python3 -c "import sys, json; print(json.dumps(json.loads(sys.stdin.read()), indent=2, ensure_ascii=False))"

echo ""
echo "========================================"
echo "测试完成"
echo "========================================"
