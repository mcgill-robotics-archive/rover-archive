cmake_minimum_required(VERSION 2.8.3)
project(arm_ros_control)

find_package(catkin REQUIRED COMPONENTS
  controller_interface
  transmission_interface
  controller_manager_tests
  velocity_controllers
  controller_manager
  combined_robot_hw
  combined_robot_hw_tests
  hardware_interface
  diff_drive_controller
  controller_manager_msgs
  realtime_tools
  position_controllers
  joint_state_controller
  gripper_action_controller
  gazebo_ros_control
  joint_trajectory_controller
  forward_command_controller
  joint_limits_interface
  imu_sensor_controller
  force_torque_sensor_controller
  rqt_joint_trajectory_controller
  effort_controllers
)

catkin_package(
  CATKIN_DEPENDS
  controller_manager_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
