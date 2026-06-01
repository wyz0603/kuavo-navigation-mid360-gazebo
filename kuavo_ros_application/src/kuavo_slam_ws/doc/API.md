# Kuavo SLAM 系统 API 文档

## 概述

本文档描述了 Kuavo SLAM 系统的所有 API 接口，包括服务（Services）、话题（Topics）、动作（Actions）等。该系统基于 ROS1 构建，集成了 SLAM、导航、地图管理等功能。

## 目录

- [导航控制接口](#导航控制接口)
- [地图管理服务](#地图管理服务)
- [任务点管理服务](#任务点管理服务)
- [全局定位服务](#全局定位服务)
- [状态监控话题](#状态监控话题)

---

## 导航控制接口

### 1. 开始导航

#### `/move_base/goal`

**接口类型**: `move_base_msgs/MoveBaseActionGoal`

**功能描述**: 发送导航目标，开始导航任务

**字段说明**:

| 字段名称 | 数据类型 | 含义 |
|----------|----------|------|
| `goal.target_pose.header.frame_id` | `string` | 目标坐标系（通常为 "map"） |
| `goal.target_pose.pose.position.x` | `float64` | 目标位置 X 坐标（米） |
| `goal.target_pose.pose.position.y` | `float64` | 目标位置 Y 坐标（米） |
| `goal.target_pose.pose.position.z` | `float64` | 目标位置 Z 坐标（米） |
| `goal.target_pose.pose.orientation.x` | `float64` | 目标朝向 X 分量 |
| `goal.target_pose.pose.orientation.y` | `float64` | 目标朝向 Y 分量 |
| `goal.target_pose.pose.orientation.z` | `float64` | 目标朝向 Z 分量 |
| `goal.target_pose.pose.orientation.w` | `float64` | 目标朝向 W 分量 |

**使用示例**:

```bash
rostopic pub /move_base/goal move_base_msgs/MoveBaseActionGoal "header:
  seq: 0
  stamp: {secs: 0, nsecs: 0}
  frame_id: ''
goal_id:
  stamp: {secs: 0, nsecs: 0}
  id: ''
goal:
  target_pose:
    header:
      seq: 0
      stamp: {secs: 0, nsecs: 0}
      frame_id: 'map'
    pose:
      position: {x: 1.0, y: 2.0, z: 0.0}
      orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}"
```

### 2. 导航状态监控

#### `/move_base/status`

**接口类型**: `actionlib_msgs/GoalStatusArray`

**功能描述**: 查看当前导航目标的执行状态

**字段说明**:

| 字段名称 | 数据类型 | 含义 |
|----------|----------|------|
| `status_list[].goal_id.id` | `string` | 目标ID |
| `status_list[].status` | `uint8` | 状态码 |
| `status_list[].text` | `string` | 状态描述文本 |

**状态码说明**:

| 状态码 | 状态名称 | 含义 |
|--------|----------|------|
| 0 | PENDING | 等待中 |
| 1 | ACTIVE | 正在执行 |
| 2 | PREEMPTED | 被取消（成功取消） |
| 3 | SUCCEEDED | 导航成功完成 |
| 4 | ABORTED | 导航失败 |
| 5 | REJECTED | 被拒绝执行 |

**使用示例**:

```bash
rostopic echo /move_base/status
```

**示例输出**:

```yaml
status_list:
- goal_id:
    id: "goal_2025-06-26-13-45-00"
  status: 2
  text: "Goal canceled."
```

**⚠️ 重要提醒**: 虽然 `status_list` 是一个数组，但在实际使用中通常只需要检索第一个元素（`status_list[0]`）即可，因为系统通常只处理一个导航目标。

### 3. 取消导航

#### `/move_base/cancel`

**接口类型**: `actionlib_msgs/GoalID`

**功能描述**: 取消当前正在执行的导航任务或所有等待中的目标

**字段说明**:

| 字段名称 | 数据类型 | 含义 |
|----------|----------|------|
| `id` | `string` | 目标ID（空字符串表示取消所有目标） |

**使用方式**:

##### 取消所有导航目标

```bash
rostopic pub /move_base/cancel actionlib_msgs/GoalID '{}'
```

---

## 地图管理服务

### `/load_map`

**服务类型**: `kuavo_mapping/LoadMap`

**功能描述**: 加载指定的地图文件

**请求参数**:

| 字段名称 | 数据类型 | 含义 |
|----------|----------|------|
| `map_name` | `string` | 地图名称 |

**响应参数**:

| 字段名称 | 数据类型 | 含义 |
|----------|----------|------|
| `success` | `bool` | 加载成功状态 |
| `message` | `string` | 状态信息 |

**使用示例**:

```bash
rosservice call /load_map "map_name: 'map_2025-01-15_10-30-00'"
```

### `/get_current_map`

**服务类型**: `kuavo_mapping/GetCurrentMap`

**功能描述**: 获取当前加载的地图名称

**请求参数**: 无

**响应参数**:

| 字段名称 | 数据类型 | 含义 |
|----------|----------|------|
| `current_map` | `string` | 当前加载的地图名称 |

**使用示例**:

```bash
rosservice call /get_current_map
```

### `/get_all_maps`

**服务类型**: `kuavo_mapping/GetAllMaps`

**功能描述**: 获取所有可用的地图列表

**请求参数**: 无

**响应参数**:

| 字段名称 | 数据类型 | 含义 |
|----------|----------|------|
| `maps` | `string[]` | 所有可用地图名称列表 |

**使用示例**:

```bash
rosservice call /get_all_maps
```

---

## 任务点管理服务

### `/task_point`

**服务类型**: `kuavo_mapping/TaskPointOperation`

**功能描述**: 任务点的增删改查操作

**操作类型常量**:

| 常量值 | 操作名称 | 含义 |
|--------|----------|------|
| 0 | ADD | 添加任务点 |
| 1 | UPDATE | 更新任务点 |
| 2 | DELETE | 删除任务点 |
| 3 | GET | 获取所有任务点 |

**请求参数**:

| 字段名称 | 数据类型 | 必填 | 含义 |
|----------|----------|------|------|
| `operation` | `int8` | ✅ | 操作类型（0-3） |
| `task_point.pose.position.x` | `float64` | ⚠️ | 任务点X坐标（米），ADD/UPDATE时必填，除非use_robot_current_pose=true |
| `task_point.pose.position.y` | `float64` | ⚠️ | 任务点Y坐标（米），ADD/UPDATE时必填，除非use_robot_current_pose=true |
| `task_point.pose.position.z` | `float64` | ⚠️ | 任务点Z坐标（米），ADD/UPDATE时必填，除非use_robot_current_pose=true |
| `task_point.pose.orientation.x` | `float64` | ⚠️ | 任务点朝向X分量，ADD/UPDATE时必填，除非use_robot_current_pose=true |
| `task_point.pose.orientation.y` | `float64` | ⚠️ | 任务点朝向Y分量，ADD/UPDATE时必填，除非use_robot_current_pose=true |
| `task_point.pose.orientation.z` | `float64` | ⚠️ | 任务点朝向Z分量，ADD/UPDATE时必填，除非use_robot_current_pose=true |
| `task_point.pose.orientation.w` | `float64` | ⚠️ | 任务点朝向W分量，ADD/UPDATE时必填，除非use_robot_current_pose=true |
| `task_point.name` | `string` | ⚠️ | 任务点名称，ADD时可选，UPDATE/DELETE时必填 |
| `name` | `string` | ⚠️ | 任务点名称（用于操作），ADD时可选，UPDATE/DELETE时必填 |
| `use_robot_current_pose` | `bool` | ❌ | 是否使用机器人当前位置(用于ADD/UPDATE操作)，默认false |

**响应参数**:

| 字段名称 | 数据类型 | 含义 |
|----------|----------|------|
| `success` | `bool` | 操作成功状态 |
| `message` | `string` | 状态信息 |
| `task_points[].pose.position.x` | `float64` | 任务点X坐标（GET操作返回） |
| `task_points[].pose.position.y` | `float64` | 任务点Y坐标（GET操作返回） |
| `task_points[].pose.position.z` | `float64` | 任务点Z坐标（GET操作返回） |
| `task_points[].pose.orientation.x` | `float64` | 任务点朝向X分量（GET操作返回） |
| `task_points[].pose.orientation.y` | `float64` | 任务点朝向Y分量（GET操作返回） |
| `task_points[].pose.orientation.z` | `float64` | 任务点朝向Z分量（GET操作返回） |
| `task_points[].pose.orientation.w` | `float64` | 任务点朝向W分量（GET操作返回） |
| `task_points[].name` | `string` | 任务点名称（GET操作返回） |

**操作说明**:

#### ADD 操作（operation=0）
- **name**: 可选，如果不提供则使用默认名称
- **use_robot_current_pose**: 如果为 `true`，则忽略 `task_point.pose` 字段，使用机器人当前位置
- **task_point.pose**: 当 `use_robot_current_pose=false` 时必填

#### UPDATE 操作（operation=1）
- **name**: 必填，指定要更新的任务点名称
- **use_robot_current_pose**: 如果为 `true`，则忽略 `task_point.pose` 字段，使用机器人当前位置
- **task_point.pose**: 当 `use_robot_current_pose=false` 时必填

#### DELETE 操作（operation=2）
- **name**: 必填，指定要删除的任务点名称
- 其他字段忽略

#### GET 操作（operation=3）
- 所有字段忽略，返回所有任务点列表

**使用示例**:

#### 添加任务点（使用指定位置）
```bash
rosservice call /task_point "operation: 0
task_point:
  pose:
    position: {x: 1.0, y: 2.0, z: 0.0}
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  name: 'home_position'
name: 'home_position'
use_robot_current_pose: false"
```

#### 添加任务点（使用机器人当前位置）
```bash
rosservice call /task_point "operation: 0
task_point:
  pose:
    position: {x: 0.0, y: 0.0, z: 0.0}
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  name: 'current_position'
name: 'current_position'
use_robot_current_pose: true"
```

#### 更新任务点
```bash
rosservice call /task_point "operation: 1
task_point:
  pose:
    position: {x: 2.0, y: 3.0, z: 0.0}
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  name: 'home_position'
name: 'home_position'
use_robot_current_pose: false"
```

#### 删除任务点
```bash
rosservice call /task_point "operation: 2
task_point:
  pose:
    position: {x: 0.0, y: 0.0, z: 0.0}
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  name: 'home_position'
name: 'home_position'
use_robot_current_pose: false"
```

#### 获取所有任务点
```bash
rosservice call /task_point "operation: 3
task_point:
  pose:
    position: {x: 0.0, y: 0.0, z: 0.0}
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  name: ''
name: ''
use_robot_current_pose: false"
```

### `/navigate_to_task_point`

**服务类型**: `kuavo_mapping/NavigateToTaskPoint`

**功能描述**: 导航到指定的任务点

**请求参数**:

| 字段名称 | 数据类型 | 含义 |
|----------|----------|------|
| `task_name` | `string` | 任务点名称 |

**响应参数**:

| 字段名称 | 数据类型 | 含义 |
|----------|----------|------|
| `success` | `bool` | 导航启动状态 |
| `message` | `string` | 状态信息 |

**使用示例**:

```bash
rosservice call /navigate_to_task_point "task_name: 'home_position'"
```

---

## 全局定位服务

### `/initialpose_with_taskpoint`

**服务类型**: `kuavo_mapping/InitialPoseWithTaskPoint`

**功能描述**: 基于任务点的全局定位

**请求参数**:

| 字段名称 | 数据类型 | 含义 |
|----------|----------|------|
| `task_point_name` | `string` | 任务点名称 |

**响应参数**:

| 字段名称 | 数据类型 | 含义 |
|----------|----------|------|
| `success` | `bool` | 操作成功状态 |
| `message` | `string` | 状态信息 |

**使用方式**: 接收任务点名称，根据任务点名称查找对应的任务点位置，自动调用 `set_initialpose` 服务进行全局定位

**使用示例**:

```bash
# 通过任务点名称进行定位
rosservice call /initialpose_with_taskpoint "task_point_name: 'task1'"
```

---

## 注意事项

⚠️ **重要提醒**：

### 建图相关

- 建图时，机器人的雷达一定要水平与地面，如果倾斜会导致建图失败
- 建议将机器人头部电机摆好后，机器人程序进入 cali_arm 模式，头部电机使能后进行建图
- 建图时用户应站在机器人身后，避免遮挡激光雷达视野
- 确保环境中没有动态物体，建图期间环境应保持相对静止
- 编辑地图前建议备份原始文件，编辑时注意保持数据一致性

### 定位和地图切换

- **⚠️ 重要限制**: 如果已经给出 `initial pose` 后再进行切换地图（load_map），就无法再次进行 global localization
- 建议在在确定使用的地图后，再给出 `initial pose`

### 任务点管理

- 任务点名称在同一地图内必须唯一
- 使用 `use_robot_current_pose=true` 时，确保机器人位置准确
- 删除任务点操作不可恢复，请谨慎操作

### 全局定位服务

- `initialpose_with_taskpoint` 服务由 `map_manager` 节点提供，用于通过任务点名称设置初始位姿
- `set_initialpose` 服务由 `global_localization` 节点提供，用于直接设置初始位姿
- 服务调用流程：`initialpose_with_taskpoint` → `map_manager` 查找任务点 → 调用 `set_initialpose` → `global_localization` 处理定位
- 服务调用是同步的，会等待操作完成才返回结果

---

## 故障排除

### 常见问题

1. **PCD文件不存在**: 检查地图构建是否完成，等待足够时间让FAST-LIO生成点云数据
2. **导航失败**: 检查地图文件路径是否正确，确保坐标系配置正确
3. **传感器数据异常**: 检查激光雷达连接和驱动配置
4. **定位失败**: 确保全局定位已完成，检查TF变换是否正确
5. **任务点操作失败**: 检查 Map Manager 是否正常运行，数据库权限是否正确
6. **地图切换后无法定位**: 切换地图后需要重新进行全局定位，如果之前已经设置过 initial pose，可能需要重启导航节点
7. **initialpose_with_taskpoint 服务调用失败**: 
   - 检查任务点名称是否存在
   - 确保 `map_manager` 节点正常运行
   - 检查 `global_localization` 节点是否启动并提供 `set_initialpose` 服务
8. **set_initialpose 服务调用失败**:
   - 确保 `global_localization` 节点正常运行
   - 检查位姿数据格式是否正确
   - 确保坐标系设置正确（通常为 "map"）
