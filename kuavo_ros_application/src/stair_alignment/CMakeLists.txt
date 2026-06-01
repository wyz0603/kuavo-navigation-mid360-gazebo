cmake_minimum_required(VERSION 3.0.2)
project(stair_alignment)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  message_generation
  geometry_msgs
  trajectory_msgs
  ocs2_msgs
  dynamic_biped
  apriltag_ros
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  StairAlignmentStatus.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  stairAlignmentSrv.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  sensor_msgs
  std_msgs
  geometry_msgs
  trajectory_msgs
  ocs2_msgs
  dynamic_biped
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS rospy message_runtime std_msgs geometry_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(PROGRAMS
  scripts/stair_alignment_node.py
  scripts/test_stair_alignment.py
  scripts/example_client.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
