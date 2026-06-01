#!/bin/bash

# 设置颜色输出
RED='\033[0;31m'
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[0;33m'
NC='\033[0m' # 无颜色

echo -e "${BLUE}欢迎使用VR测试脚本${NC}"
echo -e "${YELLOW}请选择手臂类型:${NC}"
echo "1. 长手臂(45)"
echo "2. 短手臂(42)"
read -p "请输入选择 (1/2): " arm_choice

# 根据选择设置搜索模式
if [ "$arm_choice" = "1" ]; then
    echo -e "${GREEN}已选择长手臂(45)${NC}"
    search_pattern="VR_action_new*"
elif [ "$arm_choice" = "2" ]; then
    echo -e "${GREEN}已选择短手臂(42)${NC}"
    search_pattern="VR_short_action_new*"
else
    echo -e "${RED}无效选择，退出程序${NC}"
    exit 1
fi

# 获取当前目录
current_dir=$(pwd)
echo -e "${BLUE}当前目录: $current_dir${NC}"

# 查找匹配的目录
echo -e "${BLUE}查找匹配的目录: $search_pattern${NC}"
directories=$(find "$current_dir" -type d -name "$search_pattern" | sort)

# 检查是否找到目录
if [ -z "$directories" ]; then
    echo -e "${RED}未找到匹配的目录，退出程序${NC}"
    exit 1
fi
# 显示找到的目录
echo -e "${GREEN}找到以下目录:${NC}"
echo "$directories"

# 获取循环次数参数
if [ $# -eq 0 ]; then
    # 如果没有提供参数，默认循环一次
    loop_count=1
    echo -e "${YELLOW}未指定循环次数，默认执行1次${NC}"
else
    loop_count=$1
    echo -e "${YELLOW}将执行 $loop_count 次循环${NC}"
fi

# 创建结果汇总文件
result_summary="$current_dir/vr_test_results_summary.txt"
echo "VR测试结果汇总 - $(date)" > "$result_summary"
echo "=======================================" >> "$result_summary"

# 依次处理每个目录
echo -e "${BLUE}开始依次处理每个目录...${NC}"
for dir in $directories; do
    echo -e "${YELLOW}处理目录: $dir${NC}"
    dir_name=$(basename "$dir")
    echo "动作: $dir_name" >> "$result_summary"
    
    success_count=0
    failure_count=0
    
    # 执行指定次数的循环
    for ((i=1; i<=$loop_count; i++)); do
        echo -e "${BLUE}执行第 $i/$loop_count 次测试${NC}"
        # 调用test_VR.sh脚本，传入目录路径
        is_success=1
        ./test_VR.sh "$dir" > temp_result.txt 2>&1
        test_result=$?
        
        # 提取关节误差值
        error_value=$(grep "平均关节误差" temp_result.txt | awk '{print $NF}')
        is_error=$(grep "关节动作执行" temp_result.txt | awk '{print $NF}')
        # 提取逆解误差值
        ik_error_value=$(grep "平均逆解平方差误差" temp_result.txt | awk '{print $NF}')
        is_ik_error=$(grep "逆解轨迹执行" temp_result.txt | awk '{print $NF}')
        # 检查脚本执行结果
        if [ $test_result -eq 0 ]; then
            echo -e "${GREEN}动作 $dir 第 $i 次处理完成${NC}"
            
            # 根据关节动作执行标准情况使用不同颜色显示
            if [[ "$is_error" == *"执行标准"* ]]; then
                echo "  ${GREEN}测试 #$i: 成功 - 关节误差值: $error_value, 关节动作执行标准。${NC}" >> "$result_summary"
            else
                echo "  ${RED}测试 #$i: 成功 - 关节误差值: $error_value, 关节动作执行不标准！${NC}" >> "$result_summary"
                is_success=0
            fi
            
            # 根据逆解轨迹执行标准情况使用不同颜色显示
            if [[ "$is_ik_error" == *"执行标准"* ]]; then
                echo "  ${GREEN}测试 #$i: 成功 - 逆解误差值: $ik_error_value, 逆解结果标准。${NC}" >> "$result_summary"
            else
                echo "  ${RED}测试 #$i: 成功 - 逆解误差值: $ik_error_value, 逆解结果不标准！${NC}" >> "$result_summary"
                is_success=0
            fi
            if [ $is_success == 1 ]; then  
                success_count=$((success_count+1))
            else
                failure_count=$((failure_count+1))
            fi
        else
            echo -e "${RED}动作 $dir 第 $i 次处理失败${NC}"
            
            # 根据关节动作执行标准情况使用不同颜色显示
            if [[ "$is_error" == *"执行标准"* ]]; then
                echo "  测试 #$i: 失败 - 关节误差值: $error_value, 关节动作执行标准: ${GREEN}$is_error${NC}" >> "$result_summary"
            else
                echo "  测试 #$i: 失败 - 关节误差值: $error_value, 关节动作执行标准: ${RED}$is_error${NC}" >> "$result_summary"
            fi
            
            # 根据逆解轨迹执行标准情况使用不同颜色显示
            if [[ "$is_ik_error" == *"执行标准"* ]]; then
                echo "  测试 #$i: 失败 - 逆解误差值: $ik_error_value, 逆解轨迹执行标准: ${GREEN}$is_ik_error${NC}" >> "$result_summary"
            else
                echo "  测试 #$i: 失败 - 逆解误差值: $ik_error_value, 逆解轨迹执行标准: ${RED}$is_ik_error${NC}" >> "$result_summary"
            fi
            
            failure_count=$((failure_count+1))
        fi
    done
    
    # 添加该目录的汇总信息
    echo "  汇总: 成功 $success_count 次, 失败 $failure_count 次" >> "$result_summary"
    echo "---------------------------------------" >> "$result_summary"
    echo -e "${BLUE}-----------------------------------${NC}"
done
# 删除临时文件
rm -f temp_result.txt

echo -e "${GREEN}所有目录处理完成${NC}"
echo -e "${YELLOW}结果汇总:${NC}"
while IFS= read -r line; do
    echo -e "$line"
done < "$result_summary"
