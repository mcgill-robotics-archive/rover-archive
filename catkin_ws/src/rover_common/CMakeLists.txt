cmake_minimum_required(VERSION 2.8.3)
project(rover_common)

find_package(catkin REQUIRED COMPONENTS
  genmsg
  message_generation
  roscpp
  rospy
  std_msgs
)

add_message_files(
  DIRECTORY msg
  FILES
  MotorStatus.msg
  MotorControllerMode.msg
  MagneticEncoder.msg
  DriveEncoder.msg
  DriveEncoderStamped.msg
)

add_service_files(
  DIRECTORY srv
  FILES
  GetVoltageRead.srv
  ResetWatchDog.srv
  MotorSetAngularVel.srv
  BrakeMotor.srv
  ResetMotor.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
