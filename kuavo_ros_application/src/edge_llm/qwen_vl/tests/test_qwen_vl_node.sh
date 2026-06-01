#!/bin/bash

set -o pipefail

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SERVICE_TEST_TIMEOUT="${SERVICE_TEST_TIMEOUT:-60}"
TEXT_TEST_TIMEOUT="${TEXT_TEST_TIMEOUT:-30}"
QWEN_VL_API_URL="${QWEN_VL_API_URL:-http://localhost:9000/v1/chat/completions}"

print_header() {
    echo -e "${GREEN}========================================${NC}"
    echo -e "${GREEN}$1${NC}"
    echo -e "${GREEN}========================================${NC}"
    echo
}

fail() {
    echo -e "${RED}$1${NC}"
    exit 1
}

check_ros_environment() {
    if [ -z "${ROS_DISTRO:-}" ]; then
        fail "ROS environment not sourced. Please run: source /opt/ros/<distro>/setup.bash"
    fi

    if ! rostopic list &> /dev/null; then
        fail "roscore is not running. Start roscore in another terminal."
    fi

    if ! rosservice list | grep -q "/qwen_vl_query_path"; then
        fail "Service /qwen_vl_query_path is unavailable. Launch the qwen_vl node first."
    fi

    if ! rosservice list | grep -q "/qwen_vl_query_base64"; then
        fail "Service /qwen_vl_query_base64 is unavailable. Launch the qwen_vl node first."
    fi

    echo -e "${GREEN}✓ ROS environment OK${NC}"
    echo -e "${GREEN}✓ Qwen VL services detected${NC}"
    echo
}

run_python_test() {
    local label="$1"
    shift

    print_header "$label"
    echo -e "${YELLOW}Running:${NC} $*"

    if ! "$@"; then
        echo -e "${RED}✗ ${label} failed${NC}"
        exit 1
    fi

    echo -e "${GREEN}✓ ${label} passed${NC}"
    echo
}

main() {
    print_header "Qwen VL ROS Node Test Suite"
    check_ros_environment

    run_python_test "Filesystem path service" \
        python3 "$SCRIPT_DIR/test_qwen_vl_path_service.py" \
        --timeout "$SERVICE_TEST_TIMEOUT"

    run_python_test "Base64 image service" \
        python3 "$SCRIPT_DIR/test_qwen_vl_base64_service.py" \
        --timeout "$SERVICE_TEST_TIMEOUT"

    run_python_test "Text dialogue API" \
        python3 "$SCRIPT_DIR/test_qwen_vl_text_dialog.py" \
        --api-url "$QWEN_VL_API_URL" \
        --timeout "$TEXT_TEST_TIMEOUT"

    print_header "All tests completed"
    echo -e "${BLUE}PRO TIP:${NC} Override timeouts with SERVICE_TEST_TIMEOUT=<seconds> or TEXT_TEST_TIMEOUT=<seconds>"
    echo -e "${BLUE}PRO TIP:${NC} Override API endpoint with QWEN_VL_API_URL=<url>"
}

main "$@"
