cmake_minimum_required(VERSION 2.8.3)
project(lidar_gazebo)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  gazebo_msgs
  gazebo_plugins
  gazebo_ros
  lidar_description
)

catkin_package(
  CATKIN_DEPENDS
  gazebo_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
