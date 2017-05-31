cmake_minimum_required(VERSION 2.8.3)
project(claraty_msgs)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  geometry_msgs
  nav_msgs
  std_msgs
  sensor_msgs
  message_generation
)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Arc.msg
  ArcStamped.msg
  ArcArray.msg
  FloatGrid.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  XyzToMastAzel.srv
)

# Generate actions in the 'action' folder
add_action_files(
  FILES
  MoveArc.action
  MoveTo.action
  AcquireImage.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  actionlib_msgs
  geometry_msgs
  nav_msgs
  std_msgs
  sensor_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS message_runtime
)

