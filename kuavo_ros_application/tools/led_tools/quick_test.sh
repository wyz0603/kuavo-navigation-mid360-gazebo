#!/bin/bash

# Kuavo LED 快速测试脚本 - 硬件直连模式

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LED_TEST_SCRIPT="$SCRIPT_DIR/led_comprehensive_test.py"

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${BLUE}=== Kuavo LED 快速硬件测试 ===${NC}"
echo -e "${YELLOW}硬件直连模式测试${NC}"

# 检查测试脚本是否存在
if [ ! -f "$LED_TEST_SCRIPT" ]; then
    echo -e "${RED}错误: 测试脚本不存在: $LED_TEST_SCRIPT${NC}"
    exit 1
fi

echo ""
echo "开始快速测试序列..."
echo ""

# 运行快速测试序列
echo -e "${GREEN}1. 常亮红色测试 (3秒)${NC}"
python3 "$LED_TEST_SCRIPT" --test-type single --led-mode 0 --color-preset red --duration 3

echo ""
echo -e "${GREEN}2. 呼吸绿色测试 (3秒)${NC}"  
python3 "$LED_TEST_SCRIPT" --test-type single --led-mode 1 --color-preset green --duration 3

echo ""
echo -e "${GREEN}3. 快闪蓝色测试 (3秒)${NC}"
python3 "$LED_TEST_SCRIPT" --test-type single --led-mode 2 --color-preset blue --duration 3

echo ""
echo -e "${GREEN}4. 律动彩虹测试 (3秒)${NC}"
python3 "$LED_TEST_SCRIPT" --test-type single --led-mode 3 --color-preset rainbow --duration 3

echo ""
echo -e "${GREEN}5. 关闭LED${NC}"
python3 "$LED_TEST_SCRIPT" --test-type single --led-mode 0 --color-preset off --duration 1

echo ""
echo -e "${BLUE}=== 快速测试完成 ===${NC}"
echo ""
echo "如需更多测试选项，请运行:"
echo "python3 $LED_TEST_SCRIPT --help"
