cmake_minimum_required(VERSION 2.8.3)
project(rover_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  message_runtime
  roscpp
  rospy
)

add_message_files(
  FILES
  MotorStatus.msg
  MotorControllerMode.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
)
