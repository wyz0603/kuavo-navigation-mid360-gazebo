#!/bin/bash

# --- Script Configuration ---
DEFAULT_INPUT_FILE="targets.txt"
INPUT_FILE="${1:-$DEFAULT_INPUT_FILE}" # Use first argument or default

RESULTS_DIR="c_lint_results"
GLOBAL_REPORT_FILE="$RESULTS_DIR/clang_tidy_from_list_report.txt"
WORKSPACE_ROOT=$(pwd) # Assuming script is run from workspace root

# --- Whitelist Configuration (Similar to c_lint_tool.sh) ---
WHITELIST_FILE="c_lint_whitelist.txt"
declare -a processed_whitelist_entries
if [ -f "$WHITELIST_FILE" ]; then
    echo "Reading whitelist from: $WORKSPACE_ROOT/$WHITELIST_FILE"
    mapfile -t raw_whitelist_entries < "$WHITELIST_FILE"
    for entry in "${raw_whitelist_entries[@]}"; do
        if [[ -z "$entry" || "$entry" == \#* ]]; then continue; fi
        processed_whitelist_entries+=("$entry")
    done
    if [ ${#processed_whitelist_entries[@]} -gt 0 ]; then
        echo "Processed ${#processed_whitelist_entries[@]} whitelist keywords:"
        printf "  - %s\n" "${processed_whitelist_entries[@]}"
    fi
else
    echo "Whitelist file not found at '$WORKSPACE_ROOT/$WHITELIST_FILE'. Proceeding without whitelist."
fi
echo ""
# --- End Whitelist Configuration ---

# --- Argument Check ---
if [ ! -f "$INPUT_FILE" ]; then
    echo "错误: 输入文件 '$INPUT_FILE' 未找到。" >&2
    echo "用法: $0 [path_to_targets.txt]" >&2
    exit 1
fi
echo "从 '$INPUT_FILE' 读取目标..."

# --- Prepare Results Directory and Report File ---
mkdir -p "$RESULTS_DIR"
echo "Clang-Tidy Report (from list) - $(date)" > "$GLOBAL_REPORT_FILE"
echo -e "\n创建目录用于 Clang-Tidy 结果: $WORKSPACE_ROOT/$RESULTS_DIR"
echo "所有 Clang-Tidy 输出将汇总到: $GLOBAL_REPORT_FILE"

# --- Arrays to store parsed data ---
declare -a all_compile_db_paths
declare -a all_target_files_absolute

# --- Read and Parse Input File ---
echo -e "\n解析输入文件并定位文件..."
line_num=0
while IFS= read -r line || [[ -n "$line" ]]; do
    ((line_num++))
    trimmed_line=$(echo "$line" | awk '{$1=$1};1') # Trim whitespace

    if [[ -z "$trimmed_line" || "$trimmed_line" == \#* ]]; then # Skip empty lines and comments
        continue
    fi

    # Split the line into package_name and relative_path
    # Assuming space первой is the delimiter. For paths with spaces, this will need adjustment.
    pkg_name=$(echo "$trimmed_line" | awk '{print $1}')
    relative_path=$(echo "$trimmed_line" | awk '{print $2}') # This gets the second field

    if [ -z "$pkg_name" ] || [ -z "$relative_path" ]; then
        echo "警告 (行 $line_num): 格式错误，跳过: '$line'" >&2
        continue
    fi

    pkg_root_path=$(catkin locate -s "$pkg_name" 2>/dev/null)
    if [ -z "$pkg_root_path" ]; then
        echo "警告 (行 $line_num): 无法定位功能包 '$pkg_name'，跳过条目。" >&2
        continue
    fi

    # Construct compile database path (assuming build/<pkg_name> in workspace root)
    # Ensure WORKSPACE_ROOT is correctly defined, e.g., by running the script from WS root
    compile_db_path_for_pkg="$WORKSPACE_ROOT/build/$pkg_name"

    absolute_target_path_temp="$pkg_root_path/$relative_path"
    # Normalize path to remove . and .. and make it absolute
    absolute_target_path_normalized=$(realpath -m "$absolute_target_path_temp")


    if [ -f "$absolute_target_path_normalized" ]; then
        all_target_files_absolute+=("$absolute_target_path_normalized")
        all_compile_db_paths+=("$compile_db_path_for_pkg")
        echo "  文件: $absolute_target_path_normalized (DB: $compile_db_path_for_pkg)"
    elif [ -d "$absolute_target_path_normalized" ]; then
        echo "  目录: $absolute_target_path_normalized (DB: $compile_db_path_for_pkg) - 扫描中..."
        # Find files in directory. Use -print0 and read -d $'\0' for safe handling of filenames with spaces etc.
        find "$absolute_target_path_normalized" -type f \( -name "*.c" -o -name "*.cpp" -o -name "*.cc" -o -name "*.h" -o -name "*.hpp" \) -print0 | while IFS= read -r -d $'\0' found_file; do
            found_file_abs=$(realpath -m "$found_file") # Ensure absolute and normalized
            all_target_files_absolute+=("$found_file_abs")
            all_compile_db_paths+=("$compile_db_path_for_pkg")
            echo "    发现文件: $found_file_abs"
        done
    else
        echo "警告 (行 $line_num): 路径 '$absolute_target_path_normalized' (来自 '$relative_path' in '$pkg_name') 既不是文件也不是目录。" >&2
    fi
done < "$INPUT_FILE"

if [ ${#all_target_files_absolute[@]} -eq 0 ]; then
    echo "错误: 未从输入文件 '$INPUT_FILE' 收集到任何有效的目标文件。" >&2
    exit 1
fi

# --- (Optional) Whitelist Filtering ---
declare -a filtered_target_files
declare -a filtered_compile_db_paths

echo -e "\n根据白名单过滤文件..."
num_total_files=${#all_target_files_absolute[@]}
for i in "${!all_target_files_absolute[@]}"; do
    file_to_check="${all_target_files_absolute[$i]}"
    db_path_for_file="${all_compile_db_paths[$i]}"
    is_whitelisted=false

    if [ ${#processed_whitelist_entries[@]} -gt 0 ]; then
        for wl_entry in "${processed_whitelist_entries[@]}"; do
            if [[ "$file_to_check" == *"$wl_entry"* ]]; then
                is_whitelisted=true
                echo "  跳过 (白名单): $file_to_check (关键词: '$wl_entry')"
                break
            fi
        done
    fi

    if [ "$is_whitelisted" = false ]; then
        filtered_target_files+=("$file_to_check")
        filtered_compile_db_paths+=("$db_path_for_file")
    fi
done
num_filtered_files=${#filtered_target_files[@]}
echo "$((num_total_files - num_filtered_files)) 个文件因白名单被跳过。将处理 $num_filtered_files 个文件。"


if [ ${#filtered_target_files[@]} -eq 0 ]; then
    echo "错误: 白名单过滤后，没有剩余文件可供检查。" >&2
    exit 0 # Or 1, depending on desired behavior
fi


# --- Group files by compile_db_path and run clang-tidy ---
echo -e "\n按编译数据库分组并运行 clang-tidy..."
declare -A files_by_db # Associative array: key=db_path, value=newline-separated list of files

for i in "${!filtered_target_files[@]}"; do
    db_path="${filtered_compile_db_paths[$i]}"
    file_path="${filtered_target_files[$i]}"

    if [[ -v files_by_db["$db_path"] ]]; then
        files_by_db["$db_path"]+=$'\n'"$file_path"
    else
        files_by_db["$db_path"]="$file_path"
    fi
done

overall_tidy_success=true

for db_path_key in "${!files_by_db[@]}"; do
    echo -e "\n  处理编译数据库: $db_path_key"
    
    # Convert newline-separated string of files back to an array for clang-tidy
    mapfile -t current_files_for_tidy < <(echo -e "${files_by_db["$db_path_key"]}")

    if [ ${#current_files_for_tidy[@]} -eq 0 ]; then
        echo "    此编译数据库没有文件需要检查 (可能都被白名单过滤了)。"
        continue
    fi

    echo "    将使用编译数据库 '$db_path_key' 检查以下 ${#current_files_for_tidy[@]} 个文件:"
    # printf "      %s\n" "${current_files_for_tidy[@]}" # Can be verbose

    # Append header to global report
    echo -e "\n\n################################################################################" >> "$GLOBAL_REPORT_FILE"
    echo -e "# 使用编译数据库检查: $db_path_key" >> "$GLOBAL_REPORT_FILE"
    echo -e "################################################################################\n" >> "$GLOBAL_REPORT_FILE"

    # Run clang-tidy
    if clang-tidy -p "$db_path_key" "${current_files_for_tidy[@]}" >> "$GLOBAL_REPORT_FILE" 2>&1; then
        echo "    成功运行 clang-tidy。输出已追加到 $GLOBAL_REPORT_FILE."
        echo -e "\n--- clang-tidy for $db_path_key COMPLETED SUCCESSFULLY (Exit Code: $?) ---\n" >> "$GLOBAL_REPORT_FILE"
    else
        exit_code=$?
        echo "    运行 clang-tidy 时出错或发现问题 (数据库: $db_path_key)。退出码: $exit_code"
        echo "    输出和错误已追加到 $GLOBAL_REPORT_FILE"
        echo -e "\n--- ERROR running clang-tidy for $db_path_key (Exit Code: $exit_code) ---\n" >> "$GLOBAL_REPORT_FILE"
        overall_tidy_success=false # Mark that at least one tidy command failed
    fi
done

echo -e "\nClang-tidy 检查完成。"
echo "所有结果已汇总到: $GLOBAL_REPORT_FILE"

if [ "$overall_tidy_success" = true ]; then
    exit 0
else
    echo "在 clang-tidy 执行过程中至少发生了一个错误。" >&2
    exit 1
fi