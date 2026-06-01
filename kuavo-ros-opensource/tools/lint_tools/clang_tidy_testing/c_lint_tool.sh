#!/bin/bash

# 假设你的catkin工作空间已经设置好，并且catkin命令可用


# 获取当前脚本的路径
script_path="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
echo "当前脚本路径: $script_path"

# 获取当前ROS编译的工作空间目录的路径，
workspace_dir="$(catkin locate)"
echo "当前ROS工作空间目录: $workspace_dir"



# --- Whitelist Configuration ---
whitelist_file="c_lint_whitelist.txt"
declare -a processed_whitelist_entries
script_root=$(pwd) # Assuming script is run from workspace root

if [ -f "$whitelist_file" ]; then
    echo "Reading whitelist from: $script_root/$whitelist_file"
    mapfile -t raw_whitelist_entries < "$whitelist_file"

    for entry in "${raw_whitelist_entries[@]}"; do
        # Skip empty lines or comments
        if [[ -z "$entry" || "$entry" == \#* ]]; then
            continue
        fi
        # Store the entry directly as a keyword
        processed_whitelist_entries+=("$entry")
    done
    if [ ${#processed_whitelist_entries[@]} -gt 0 ]; then
        echo "Processed ${#processed_whitelist_entries[@]} whitelist keywords:"
        printf "  - %s\n" "${processed_whitelist_entries[@]}"
    else
        echo "Whitelist file '$whitelist_file' is empty or contains only comments."
    fi
else
    echo "Whitelist file not found at '$script_root/$whitelist_file'. Proceeding without whitelist."
fi
echo "" # Newline for clarity
# --- End Whitelist Configuration ---





# 1. 捕获 catkin build --dry-run 的输出中的功能包名称
#    使用 grep 和 awk 提取 "Packages to be built:" 下面的行
#    并去除可能存在的 "[build - ...]" 或 "[...]" 等前缀
#    注意：这里假设 'Packages to be built:' 这一行之后，每个包名独占一行
#    且没有其他干扰信息直到列表结束。
package_names_output=$(catkin build humanoid_controllers hardware_node humanoid_interface humanoid_estimation kuavo_estimation humanoid_interface_drake humanoid_wbc --dry-run 2>&1)

# 使用awk来提取包名
# 这里我们寻找 "Packages to be built:" 之后的所有行，并过滤掉空行
# 并且去除行首的 "- " 和任何 "[...]" 类似的信息
# 确保在实际环境中测试这个正则表达式，以防 catkin build 的输出格式有所变化
readarray -t package_names < <(echo "$package_names_output" | \
    awk '/Packages to be built:/ {p=1; next} p && /^- / {sub(/^- /, ""); sub(/\s*\[.*\]\s*$/, ""); sub(/\s*\(catkin\)\s*$/, ""); gsub(/[ \t]+$/, ""); print}' | \
    grep -v '^$') # 过滤空行

# 检查是否成功获取了包名
if [ ${#package_names[@]} -eq 0 ]; then
    echo "Error: No packages found in the dry-run output. Please check the command and its output."
    exit 1
fi

echo "Discovered packages (${#package_names[@]}):"
printf "  - %s\n" "${package_names[@]}"

# 2. 遍历功能包名称，获取其路径，并存储在一个新数组中
declare -a package_paths # 声明一个数组来存储路径

echo -e "\nLocating package paths..."
for pkg_name in "${package_names[@]}"; do
    pkg_path=$(catkin locate -s "$pkg_name" 2>/dev/null) # 2>/dev/null 隐藏错误输出
    if [ -n "$pkg_path" ]; then # 检查路径是否非空
        package_paths+=("$pkg_path")
        echo "  - $pkg_name: $pkg_path"
    else
        echo "  - Warning: Could not locate path for package: $pkg_name"
    fi
done

# echo -e "\nAll discovered package paths (${#package_paths[@]}):"
# printf "  - %s\n" "${package_paths[@]}"


# 3. Create a directory for lint results in the workspace root
#    Assuming this script is run from the catkin workspace root.
results_dir="c_lint_results"
mkdir -p "$results_dir"
global_report_file="$results_dir/clang_tidy_report.txt" # Define the global report file
echo "Clang-Tidy Report - $(date)" > "$global_report_file"  # Initialize/clear the global report file
echo -e "\nCreated directory for Clang-Tidy results: $(pwd)/$results_dir"
echo "All Clang-Tidy output will be into: $global_report_file" # Updated message

# 4. Run clang-tidy for each package
echo -e "\nRunning clang-tidy for each discovered package..."
for i in "${!package_names[@]}"; do
    pkg_name="${package_names[$i]}"
    pkg_path="${package_paths[$i]}" # Absolute path to package source

    echo "  Processing package: $pkg_name (Source: $pkg_path)"

    # Check if the package name is in the processed whitelist entries
    if [[ " ${processed_whitelist_entries[@]} " =~ " ${pkg_name} " ]]; then
        echo "    Skipping package $pkg_name as it is in the processed whitelist."
        continue
    fi

    # Find all C/C++ source files in the package directory
    package_all_source_files=()
    while IFS= read -r line; do
        # Ensure we are not adding empty lines if find returns nothing and the loop runs once
        if [ -n "$line" ]; then
            # Convert to absolute path right here for consistency
            package_all_source_files+=("$(realpath -m "$line")") # Ensure files are absolute paths
        fi
    done < <(find "$pkg_path" -type f \( -name "*.c" -o -name "*.cpp" -o -name "*.cc" \) )

    # Filter source files based on whitelist
    source_files_list=()
    if [ ${#package_all_source_files[@]} -gt 0 ]; then
        echo "    Found ${#package_all_source_files[@]} potential source file(s) for $pkg_name. Filtering based on whitelist keywords..." # MODIFIED
        for file_to_check in "${package_all_source_files[@]}"; do # file_to_check is already an absolute path
            
            # 检查文件路径中是否含由白名单中的关键词，如果有，则跳过
            is_whitelisted=false
            if [ ${#processed_whitelist_entries[@]} -gt 0 ]; then
                for wl_entry in "${processed_whitelist_entries[@]}"; do
                    # Check if file_to_check (absolute path) contains the wl_entry keyword
                    if [[ "$file_to_check" == *"$wl_entry"* ]]; then # MODIFIED
                        is_whitelisted=true
                        echo "      Skipping (whitelisted): $file_to_check (path contains keyword: '$wl_entry')" # MODIFIED
                        break
                    fi
                done
            fi

            if [ "$is_whitelisted" = false ]; then
                source_files_list+=("$file_to_check")
            fi
        done
        echo "    After whitelist filtering, ${#source_files_list[@]} source file(s) will be processed for $pkg_name."
    fi

    if [ ${#source_files_list[@]} -eq 0 ]; then
        echo "    No source files (.c, .cpp, .cc) found for $pkg_name in $pkg_path after applying whitelist (or none found initially). Skipping clang-tidy."
        continue
    fi

    echo "    Found ${#source_files_list[@]} source file(s) for $pkg_name."
    # To print files for debugging: printf "      %s\n" "${source_files_list[@]}"

    # Append a separator and package info to the global report file
    echo -e "\n\n################################################################################" >> "$global_report_file"
    echo -e "# Checking package: $pkg_name" >> "$global_report_file"
    echo -e "# Source: $pkg_path" >> "$global_report_file"
    # To include the list of files in the report:
    # echo -e "# Files: ${source_files_list[*]}" >> "$global_report_file"
    echo -e "################################################################################\n" >> "$global_report_file"

    clang_tidy_compile_commands_path_arg="build/$pkg_name"

    echo "    Using compilation database path for clang-tidy: $clang_tidy_compile_commands_path_arg"

    # Run clang-tidy, appending both stdout and stderr to the global report file
    if clang-tidy -p "$clang_tidy_compile_commands_path_arg" "${source_files_list[@]}" >> "$global_report_file" 2>&1; then
        echo "    Successfully ran clang-tidy for $pkg_name. Output appended to $global_report_file."
        # Optionally add a success marker to the report file itself
        echo -e "\n--- clang-tidy for $pkg_name completed successfully (Exit Code: $?) ---\n" >> "$global_report_file"
    else
        # $? will hold the exit code of clang-tidy
        echo "    Error or issues running clang-tidy for $pkg_name. Exit code: $?"
        echo "    Output and errors (if any) for $pkg_name have been appended to $global_report_file"
        # Add an error marker to the global report file as well
        echo -e "\n--- ERROR running clang-tidy for $pkg_name (Exit Code: $?) ---\n" >> "$global_report_file"
        # Securely print the command arguments for debugging; avoid direct eval of complex strings in echo
        echo "    Command attempted (first few files): clang-tidy -p \"$clang_tidy_compile_commands_path_arg\" $(echo "${source_files_list[@]}" | cut -d ' ' -f 1-5)... >> \"$global_report_file\" 2>&1"
        echo "    Please check if the compilation database path '$clang_tidy_compile_commands_path_arg' is correct and contains necessary build information (e.g., compile_commands.json)."
        echo "    If your main compile_commands.json is in the workspace 'build' directory (e.g., $(pwd)/build/compile_commands.json), you might need to use '-p build' instead for the clang-tidy command."
    fi
    echo "" # Newline for clarity before next package
done

echo -e "\nClang-tidy checks finished."
echo "All results have been combined into: $global_report_file" # Updated final message
