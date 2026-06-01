cmake_minimum_required(VERSION 3.0.2)
project(pointcloud_converter)

## Compile as C++14, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  pcl_ros
  pcl_conversions
  tf2
  tf2_ros
  geometry_msgs
  livox_ros_driver2
)

## Find PCL
find_package(PCL REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS roscpp rospy sensor_msgs pcl_ros pcl_conversions tf2 tf2_ros geometry_msgs livox_ros_driver2
  DEPENDS PCL
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

## Add PCL definitions
add_definitions(${PCL_DEFINITIONS})

## Declare a C++ executable
add_executable(hesai_to_velodyne_converter src/hesai_to_velodyne_converter.cpp)

## Add cmake target dependencies of the executable
add_dependencies(hesai_to_velodyne_converter ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(hesai_to_velodyne_converter
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

## Livox simulation bridge (Gazebo sim PointCloud -> FAST-LIO CustomMsg + PointCloud2)
add_executable(livox_sim_bridge src/livox_sim_bridge.cpp)
add_dependencies(livox_sim_bridge ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(livox_sim_bridge
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

## Mark executables for installation
install(TARGETS hesai_to_velodyne_converter livox_sim_bridge
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
  PATTERN ".svn" EXCLUDE
)
