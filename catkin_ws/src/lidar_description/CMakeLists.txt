cmake_minimum_required(VERSION 2.8.3)
project(lidar_description)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
  urdf
  xacro
)

catkin_package(
  CATKIN_DEPENDS
  std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(state_publisher src/state_publisher.cpp)
target_link_libraries(state_publisher ${catkin_LIBRARIES})

install(TARGETS state_publisher
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
