cmake_minimum_required(VERSION 2.8.3)
project(hci)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  # ahrs
  geometry_msgs
  roscpp
  rospy
  # rover_msgs
  sensor_msgs
  std_msgs
)

