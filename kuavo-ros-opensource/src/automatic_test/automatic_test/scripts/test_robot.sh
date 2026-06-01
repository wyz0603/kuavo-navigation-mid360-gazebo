#!/bin/bash

# 默认值设置
LOG_LEVEL="WARNING"
MARKERS=()
PYTEST_ARGS=()

# 帮助信息
function show_help {
    echo "Usage: $0 [options]"
    echo "Options:"
    echo "  --log-level LEVEL    Set log level (DEBUG|INFO|WARNING|ERROR|CRITICAL)"
    echo "  --walk              Enable walk tests"
    echo "  --no-walk           Disable walk tests"
    echo "  --single-step-control Enable single step control tests"
    echo "  --no-single-step-control Disable single step control tests"
    echo "  -h, --help          Show this help message"
    echo "  -- ...              Pass remaining arguments directly to pytest"
}

# 参数解析
while [[ $# -gt 0 ]]; do
    case $1 in
        --log-level)
            LOG_LEVEL="$2"
            shift 2
            ;;
        --walk)
            MARKERS+=("walk")
            shift
            ;;
        --no-walk)
            MARKERS+=("not walk")
            shift
            ;;
        --single-step-control)
            MARKERS+=("single_step_control")
            shift
            ;;
        --no-single-step-control)
            MARKERS+=("not single_step_control")
            shift
            ;;
        -h|--help)
            show_help
            exit 0
            ;;
        --)
            shift
            PYTEST_ARGS+=("$@")
            break
            ;;
        __*|_*) # 忽略 ROS 的特殊参数
            shift
            ;;
        *)
            echo "Unknown option: $1"
            show_help
            exit 1
            ;;
    esac
done

# 构建 marker 表达式
MARKER_EXPR=""
if [ ${#MARKERS[@]} -gt 0 ]; then
    # 将所有标记用引号括起来并用 or 连接
    MARKER_EXPR="${MARKERS[0]}"
    for ((i=1; i<${#MARKERS[@]}; i++)); do
        MARKER_EXPR="$MARKER_EXPR or ${MARKERS[$i]}"
    done
fi

# 输出 marker 表达式用于调试
echo "Marker expression: $MARKER_EXPR"

ENV_PATH="$(rospack find automatic_test)/env/automatic_test.env"
source $ENV_PATH
export AUTOMATIC_TEST_WEBHOOK_URL=$AUTOMATIC_TEST_WEBHOOK_URL
export ROBOT_SERIAL_NUMBER=$ROBOT_SERIAL_NUMBER

# 构建完整的 pytest 命令
CMD="pytest -v -s --log-cli-level=$LOG_LEVEL"

if [ ! -z "$MARKER_EXPR" ]; then
    CMD="$CMD -m \"$MARKER_EXPR\""
fi

# 添加测试文件路径
TEST_PATH="$(rospack find automatic_test)/scripts/automatic_test"
CMD="$CMD $TEST_PATH"

# 添加额外的 pytest 参数
if [ ${#PYTEST_ARGS[@]} -gt 0 ]; then
    CMD="$CMD ${PYTEST_ARGS[@]}"
fi

# 输出执行信息
echo "Executing: $CMD"

# 执行命令
eval $CMD

# 保存返回值
EXIT_CODE=$?

# 退出
exit $EXIT_CODE