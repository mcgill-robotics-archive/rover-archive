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
  drive_control
  geometry_msgs
  roscpp
  rospy
  rosserial_${PROJECT_NAME}
  rosserial_client
  rover_common
  std_msgs
)

catkin_package(
    CATKIN_DEPENDS drive_control geometry_msgs message_runtime std_msgs roscpp
)

include_directories(
    ${catkin_INCLUDE_DIRS}
)

add_executable(drive_serial src/drive/drive_serial.cpp)
target_link_libraries(drive_serial ${catkin_LIBRARIES})
add_executable(arm_serial src/arm/arm_serial.cpp)
target_link_libraries(arm_serial ${catkin_LIBRARIES})
add_executable(arm_serial src/serial/arduino_serial.cpp)
target_link_libraries(arduino_serial ${catkin_LIBRARIES})

install(TARGETS
  drive_serial
  arm_serial
  arduino_serial
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
