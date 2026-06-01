cmake_minimum_required(VERSION 3.0.2)
project(motioncapture)

## 1. 找到 catkin，并指定需要用到的 ROS 组件
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  sensor_msgs
  visualization_msgs 
  message_generation
  tf2_ros
)

## 2. 查找第三方库，例如 Eigen3
find_package(Eigen3 REQUIRED)

## 3. 添加消息文件
add_message_files(
  DIRECTORY msg
  FILES
  ForcePlateData.msg
)

## 4. 生成消息
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
)

## 5. 声明 catkin 的包
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES motioncapture   # 如果有自定义库，请在此处添加
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs sensor_msgs message_runtime
  DEPENDS Eigen3
)

## 6. 设置编译选项
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -fpermissive")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -fpermissive")
set(CMAKE_CXX_FLAGS_RELEASE "-O2")
set(CMAKE_CXX_FLAGS_DEBUG "-O0")
set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -s")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -s")
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

## Linux 下定义宏
if (UNIX)
    add_definitions(-D_LINUX)
endif()

## 7. 包含目录
include_directories(
  include 
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

## 8. 库目录（如果有自定义库或第三方库）
link_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/lib
)

## 9. 添加可执行文件
add_executable(SampleClient
  src/SampleClient.cpp    # 源码放在 src/ 文件夹下
)

## 10. 链接库
target_link_libraries(SampleClient
  ${catkin_LIBRARIES}
  nokov_sdk               # 如果使用 SeekerSDK，请替换为相应库名称
  pthread
  Eigen3::Eigen
  stdc++fs                # C++11中用于filesystem支持
)

## 11. 安装可执行文件（可选，用于使用 rosrun 运行）
install(TARGETS SampleClient
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
