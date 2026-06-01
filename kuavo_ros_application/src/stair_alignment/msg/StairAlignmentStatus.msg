# 楼梯对齐状态消息
uint8 tag_id                    # 目标 AprilTag ID
string current_state            # 当前状态: "idle", "detecting", "aligning", "completed", "failed"
float64 current_x               # 当前相对于目标的 X 位置
float64 current_y               # 当前相对于目标的 Y 位置  
float64 current_yaw             # 当前相对于目标的 Yaw 角度
float64 target_x                # 目标 X 位置
float64 target_y                # 目标 Y 位置
float64 target_yaw              # 目标 Yaw 角度
uint32 step_count               # 已执行的步数
uint32 total_steps              # 总步数
string message                  # 状态描述信息
bool is_aligned                 # 是否已对齐到位
