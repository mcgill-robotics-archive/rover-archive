cmake_minimum_required(VERSION 2.8.3)
project(rover_common)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  rospy
  std_msgs
)

add_message_files(
  FILES
  MotorStatus.msg
  MotorControllerMode.msg
  Magnetic_encoder.msg
  DriveEncoder.msg
  DriveEncoderStamped.msg
)

add_service_files(
    FILES
    GetVoltageRead.srv
    ResetWatchDog.srv
    MotorSetAngularVel.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
)

catkin_package(
    CATKIN_DEPENDS message_runtime   std_msgs
)
