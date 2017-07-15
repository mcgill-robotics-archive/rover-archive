cmake_minimum_required(VERSION 2.8.3)
project(lidar)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  tf2
  tf2_ros
  urg_node
)

catkin_package()
