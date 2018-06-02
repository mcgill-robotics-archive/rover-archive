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

catkin_package(
    CATKIN_DEPENDS geometry_msgs message_runtime std_msgs roscpp
)

include_directories(
    ${catkin_INCLUDE_DIRS}
)

add_executable(drive_serial src/drive/drive_serial.cpp)
target_link_libraries(drive_serial ${catkin_LIBRARIES})

install(TARGETS
  drive_serial
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
