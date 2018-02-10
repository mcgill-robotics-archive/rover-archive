cmake_minimum_required(VERSION 2.8.3)
project(arduino)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

if(APPLE) 
  set(ahrs_pkg)
  elseif(UNIX)
  set(ahrs_pkg ahrs)
endif()
find_package(catkin REQUIRED COMPONENTS
  ${ahrs_pkg}
  ${PROJECT_NAME}_msgs
  geometry_msgs
  roscpp
  rospy
  rosserial_${PROJECT_NAME}
  rosserial_client
  rover_common
  std_msgs
)

rosserial_generate_ros_lib(
    PACKAGE rosserial_${PROJECT_NAME}
    SCRIPT make_libraries.py
)

catkin_package(
    CATKIN_DEPENDS geometry_msgs message_runtime std_msgs
)

include_directories(
    ${catkin_INCLUDE_DIRS}
)

rosserial_configure_client(
    DIRECTORY src
    TOOLCHAIN_FILE ${ROSSERIAL_ARDUINO_TOOLCHAIN}
)

rosserial_add_client_target(src drive ALL)
rosserial_add_client_target(src drive-upload)

rosserial_add_client_target(src pid_tests ALL)
rosserial_add_client_target(src pid_tests-upload)
