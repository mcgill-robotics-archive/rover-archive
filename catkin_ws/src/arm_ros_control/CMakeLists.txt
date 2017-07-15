cmake_minimum_required(VERSION 2.8.3)
project(arm_ros_control)

find_package(catkin REQUIRED COMPONENTS
  controller_interface
  controller_manager
  gazebo_ros_control
  joint_state_controller
  position_controllers
  rqt_joint_trajectory_controller
)

catkin_package(
  CATKIN_DEPENDS
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
