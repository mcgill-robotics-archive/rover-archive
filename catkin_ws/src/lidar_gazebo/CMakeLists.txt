cmake_minimum_required(VERSION 2.8.3)
project(lidar_gazebo)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  gazebo_msgs
  gazebo_plugins
  gazebo_ros
  lidar_description
  nav_msgs
  tf
  roscpp
  sensor_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Find gazebo
find_package(gazebo REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

## Find Boost
find_package(Boost REQUIRED COMPONENTS thread)
include_directories(${Boost_INCLUDE_DIRS})

catkin_package(
  CATKIN_DEPENDS
  roscpp
  gazebo_ros
  gazebo_plugins
  nav_msgs
  tf
  sensor_msgs
)

# Build our plugin
add_library(tilting_unit_gazebo_plugin plugins/tilting_unit_gazebo_plugin.cpp)
target_link_libraries(tilting_unit_gazebo_plugin ${Boost_libraries} ${GAZEBO_libraries} ${roscpp_libraries})

install(TARGETS
  tilting_unit_gazebo_plugin 
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
