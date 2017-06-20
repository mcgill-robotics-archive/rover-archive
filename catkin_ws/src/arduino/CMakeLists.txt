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
  geometry_msgs
  roscpp
  rospy
  rover_common
  rosserial_arduino
  rosserial_client
  std_msgs
  ${ahrs_pkg}
  message_generation
)

add_service_files(DIRECTORY srv
    FILES
    Ram.srv
    ResetArduino.srv
)
generate_messages(
    DEPENDENCIES
    std_msgs
)

catkin_package(
    CATKIN_DEPENDS message_runtime std_msgs
)

rosserial_generate_ros_lib(
    PACKAGE rosserial_arduino
    SCRIPT make_libraries.py
)

add_dependencies(${PROJECT_NAME}_ros_lib ${${PROJECT_NAME}_EXPORTED_TARGETS})

rosserial_configure_client(
    DIRECTORY src
    TOOLCHAIN_FILE ${ROSSERIAL_ARDUINO_TOOLCHAIN}
)

rosserial_add_client_target(src drive ALL)
# rosserial_add_client_target(drive drive-upload)
