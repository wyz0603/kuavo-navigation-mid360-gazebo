#!/bin/bash

# 默认值设置
LOG_LEVEL="WARNING"

# 帮助信息
function show_help {
    echo "Usage: $0 [options]"
    echo "Options:"
    echo "  --log-level LEVEL    Set log level (DEBUG|INFO|WARNING|ERROR|CRITICAL)"
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

# 环境设置
ENV_PATH="$(rospack find automatic_test)/env/automatic_test.env"
source $ENV_PATH
export AUTOMATIC_TEST_WEBHOOK_URL=$AUTOMATIC_TEST_WEBHOOK_URL
export ROBOT_SERIAL_NUMBER=$ROBOT_SERIAL_NUMBER

# 构建 pytest 命令
TEST_PATH="$(rospack find automatic_test)/scripts/automatic_test"
CMD="pytest -v -s --log-cli-level=$LOG_LEVEL -m single_step_control $TEST_PATH"

# 添加额外的 pytest 参数
if [ ${#PYTEST_ARGS[@]} -gt 0 ]; then
    CMD="$CMD ${PYTEST_ARGS[@]}"
fi

# 输出执行信息
echo "Executing: $CMD"

# 执行命令
eval $CMD
exit $?