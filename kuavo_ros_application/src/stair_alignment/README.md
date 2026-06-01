# 楼梯对齐节点 (Stair Alignment Node)

## 功能概述

楼梯对齐节点是一个ROS功能包，用于控制机器人通过AprilTag识别和单步控制实现对楼梯的精确对齐。

## 主要功能

### 1. 服务触发

- 提供ROS服务 `stair_alignment` 来触发对齐逻辑
- 支持动态传入AprilTag编号和相对位置参数

### 2. 参数配置

- 支持通过launch文件传入默认参数
- 支持通过服务请求动态指定参数
- 参数包括：
  - `tag_id`: AprilTag编号
  - `offset_x`: X方向偏移量 (米)
  - `offset_y`: Y方向偏移量 (米)
  - `offset_yaw`: Yaw角度偏移量 (弧度)

### 3. 状态发布

- 实时发布对齐状态到 `/stair_alignment_status` 话题
- 状态信息包括：
  - 当前状态 (idle, detecting, aligning, completed, failed)
  - 当前位置和目标位置
  - 步数进度
  - 对齐完成状态

### 4. 单步控制

- 使用单步控制实现精确移动
- 每步移动后发布状态更新
- 支持重试机制

## 文件结构

```
src/stair_alignment/
├── CMakeLists.txt                    # CMake构建文件
├── package.xml                       # 包描述文件
├── config/
│   └── config.json                   # 配置文件
├── launch/
│   └── stair_alignment.launch        # 启动文件
├── msg/
│   └── StairAlignmentStatus.msg      # 状态消息定义
├── srv/
│   └── stairAlignmentSrv.srv         # 服务定义
└── scripts/
    ├── stair_alignment_node.py       # 主节点启动脚本
    ├── stair_alignment_service.py    # 服务实现
    ├── stair_close_to_tag.py         # 控制逻辑实现
    ├── test_stair_alignment.py       # 测试脚本
    └── common/
        ├── config.py                  # 配置工具
        └── utils.py                   # 工具函数
```

## 使用方法

### 1. 启动节点

```bash
# 使用默认参数启动
roslaunch stair_alignment stair_alignment.launch

# 使用自定义参数启动
roslaunch stair_alignment stair_alignment.launch tag_id:=2 offset_x:=-0.5 offset_y:=0.1 offset_yaw:=0.1
```

### 2. 调用服务

```python
#!/usr/bin/env python
import rospy
from stair_alignment.srv import stairAlignmentSrv

rospy.init_node('test_client')
rospy.wait_for_service('stair_alignment')

try:
    align_service = rospy.ServiceProxy('stair_alignment', stairAlignmentSrv)
  
    # 创建请求
    req = stairAlignmentSrv()
    req.tag_id = 1
    req.offset_x = -0.6
    req.offset_y = 0.0
    req.offset_yaw = 0.0
  
    # 调用服务
    response = align_service(req)
  
    if response.result:
        print("对齐成功:", response.message)
    else:
        print("对齐失败:", response.message)
    
except rospy.ServiceException as e:
    print("服务调用失败:", e)
```

### 3. 监听状态

```bash
# 监听状态话题
rostopic echo /stair_alignment_status
```

### 4. 测试脚本

```bash
# 使用launch文件参数测试
rosrun stair_alignment test_stair_alignment.py launch

# 使用自定义参数测试
rosrun stair_alignment test_stair_alignment.py custom
```

## 状态消息格式

`StairAlignmentStatus` 消息包含以下字段：

```msg
uint8 tag_id                    # 目标 AprilTag ID
string current_state            # 当前状态
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
```

## 服务接口

`stairAlignmentSrv` 服务接口：

**请求 (Request):**

```srv
uint8 tag_id          # Target AprilTag ID to align to
float64 offset_x      # Expected X offset from AprilTag (meters)
float64 offset_y      # Expected Y offset from AprilTag (meters)  
float64 offset_yaw    # Expected Yaw offset from AprilTag (radians)
```

**响应 (Response):**

```srv
bool result           # true - success, false - failure
string message
```

## 配置参数

配置文件 `config/config.json` 包含以下参数：

```json
{
    "tag_id": 1,
    "xyz_offset": [0.0, 0.0, 0.0],
    "stand_params": {
        "expected_offset": [-0.6, 0.0, 0],
        "x_threshold": 0.05,
        "y_threshold": 0.05, 
        "yaw_deg_threshold": 8
    },
    "head_orientation": {
        "yaw_deg": 0,
        "pitch_deg": -10
    },
    "max_step_sizes": {
        "max_x_step": 0.04,
        "max_y_step": 0.03,
        "max_yaw_step_deg": 25
    },
    "timing_params": {
        "step_duration": 1.2,
        "wait_buffer": 2.0,
        "detection_timeout": 1.0
    }
}
```

## 工作流程

1. **初始化**: 节点启动，加载配置参数
2. **服务调用**: 客户端调用 `stair_alignment` 服务
3. **参数处理**: 使用请求参数或launch文件参数
4. **头部调整**: 调整机器人头部以便识别AprilTag
5. **标签检测**: 检测目标AprilTag
6. **单步对齐**: 使用单步控制逐步移动到目标位置
7. **状态发布**: 实时发布对齐状态
8. **完成通知**: 发布完成状态并回正头部
