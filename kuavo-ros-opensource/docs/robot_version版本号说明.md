# ROBOT_VERSION 版本号规则说明

## 版本号规则
为了兼容现有的版本以及方便后续新增，我们采用`PPPPMMMMN`的形式表达版本号，其中:
- MMMM 为主版本号(Major): 比如现有的`45`版本中的 4, 范围为: 0~9999
- N 为次版本号(Minor): 比如现有的`45`版本中的 5, 范围为: 0~9
- PPPP 为修订版本号(Patch): 比如现有的`45`版本中的 0, 范围为: 0~9999

**表达形式**:
- BIGNUMBER 整数表示为: `PPPPMMMMN`, 从个位往高位，最后一位代表次版本号 Minor, 然后依次按照规则表示主版本和修订版本，比如 `45`,`100045`...
- STRING 字符串表示为: `PPPPMMMMN`, 从个位往高位，最后一位代表次版本号 Minor, 然后依次按照规则表示主版本和修订版本，比如`"45"`,`"100045"`...

**规则说明**:
- 规则1: 表达为 BIGNUMBER/String 时，前面的 0 可以忽略,比如 45 版本我们表示为`45`而不是`000000045`
- 规则2: Patch 不为 0 时，Major 需要补全前面的 0, 比如 45.1 版本我们表示为`100045`而不是`145`
- 规则3: Patch, Major, Minor 必须要在规定的范围内，否则视为不合法的版本号

**版本号举例**:
- `4`: 表示 `0.4`的版本，因为`4=0000,0000,4`，其中主版本为0，Patch为 0，次版本为 4
- `45`: 表示 `4.5`的版本即对应 45 的机器人，因为`45=0000,0004,5`，其中主版本为4，Patch为 0，次版本为 5
- `100045`: 表示 `4.5.1`的版本即对应 45.1的机器人，因为`100045=0001,0004,5`，其中主版本为4，Patch为 1，次版本为 5
- `100100045`: 表示`4.5.1001`的版本即对应 45.1001 的机器人，因为`100100045=1001,0004,5`，其中主版本为4，Patch为 1001，次版本为 5 
- `1140000045`: 是不合法的版本号, 因为按照规则, `1140000045=11400,0004,5`，其中 Patch为 11400，超出了规定的范围

```bash
BIGBNUMBER FORMAT: PPPPMMMMN
                    |   |  |
    45   <----->   000000045
                    P   M  N
                    |   |  |
                    0   4  5  --> 45 ---> "4.5.0"

BIGBNUMBER FORMAT: PPPPMMMMN
    45.1 <----->   000100045
                    |   |  |
                    P   M  N
                    |   |  |
                    1   4  5  --> 45.1 ---> "4.5.1"

BIGBNUMBER FORMAT: PPPPMMMMN
  45.1001 <----->  100100045
                    |   |  |
                    P   M  N
                    |   |  |
                   1001 4  5  --> 45.1001 ---> "4.5.1001"

BIGBNUMBER FORMAT: PPPPMMMMN
  2045.1  <----->  000102045
                    |   |  |
                    P   M  N
                    |   |  |
                    1  204 5  --> 2045.1  ---> "204.5.1"                 
```

## 版本号情景使用

在环境变量中，版本号需统一按照 BIGNUMBER 即`PPPPMMMMN`的整数形式设置，比如:
```bash
export ROBOT_VERSION=45 # OK 
export ROBOT_VERSION=100045 # OK 45.1
export ROBOT_VERSION=10100045 # OK 45.101
export ROBOT_VERSION=1234500045 # No 不合法的版本号 Patch 12345 超出了规定的范围
```

在 rosparam 参数服务器中，版本号需统一按照 BIGNUMBER 即`PPPPMMMMN`的整数形式设置，比如:
```bash
rosparam set /robot_version  45 # OK
rosparam set /robot_version  100045 # OK 45.1 
```

配置目录命名与 URDF 文件命名时，文件名需统一按照 BIGNUMBER 即`PPPPMMMMN`的整数形式命名，比如:
```bash
# 45 <--> 45
src/kuavo_assets/models/biped_s45
src/kuavo_assets/config/kuavo_v45
src/kuavo_assets/models/biped_s45/urdf/biped_s45.urdf
src/humanoid-control/humanoid_controllers/config/kuavo_v45
# 100045 <--> 45.1 
src/kuavo_assets/models/biped_s100045
src/kuavo_assets/config/kuavo_v100045
src/kuavo_assets/models/biped_s100045/urdf/biped_s100045.urdf
src/humanoid-control/humanoid_controllers/config/kuavo_v100045
```

## For 开发人员: 在代码中如何使用版本号

### C++

建议 C++ 编写的代码使用`src/kuavo_common/include/kuavo_common/common/common.h`的 `KuavoVersion`来表述版本号, 该类提供:
- `KuavoVersion::create()`: 从 BIGNUMBER 创建版本号对象
- `KuavoVersion::is_valid()`: 可用于判断版本号是否有效
- `start_with()`: 可用于判断版本号是否属于对应的系列
- `to_string()`: 可用于将版本号转换为字符串, 比如`"100045"`
- `version_number()`: 可用于获取版本号对应的 BIGNUMBER 整型
-  ....

**从 rosparam 参数服务器初始版本号:**
```cpp
RobotVersion rb_version(4, 2);
if (nodeHandle.hasParam("/robot_version"))
{
  int rb_version_int;
  nodeHandle.getParam("/robot_version", rb_version_int);
  rb_version = RobotVersion::create(rb_version_int);
}
```

### Python

**判断是否属于 45 系列的机器人(包括基于45扩展的机器人)**:

```python
robot_version = rospy.get_param('robot_version')
if (robot_version%10000) == 45:
  print("属于 45 系列")
else:
  print("不属于 45 系列")  
```
