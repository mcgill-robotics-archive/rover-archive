cmake_minimum_required(VERSION 2.8.3)
project(lidar)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  geometry_msgs
  urg_node
  roscpp
  rospy
  std_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
  cfg/LidarTilt.cfg
)

