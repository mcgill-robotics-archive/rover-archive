cmake_minimum_required(VERSION 2.8.3)
project(arm_tiva)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  rosserial_client
  rosserial_tivac
  std_msgs
  geometry_msgs
  roscpp
  rover_common
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs
)

rosserial_generate_ros_lib(
  PACKAGE rosserial_tivac
  SCRIPT make_libraries_tiva
)

# encoder_cui
rosserial_configure_client(
  DIRECTORY encoder_cui
  TOOLCHAIN_FILE ${ROSSERIAL_TIVAC_TOOLCHAIN}
)
rosserial_add_client_target(encoder_cui encoder_cui.axf ALL)
rosserial_add_client_target(encoder_cui flash)
rosserial_add_client_target(encoder_cui size)
rosserial_add_client_target(encoder_cui dump)

# encoder_magnetic
#rosserial_configure_client(
#  DIRECTORY encoder_magnetic
#  TOOLCHAIN_FILE ${ROSSERIAL_TIVAC_TOOLCHAIN}
#)
#rosserial_add_client_target(encoder_magnetic encoder_magnetic.axf ALL)
#rosserial_add_client_target(encoder_magnetic flash)
#rosserial_add_client_target(encoder_magnetic size)
#rosserial_add_client_target(encoder_magnetic dump)

# motor
rosserial_configure_client(
  DIRECTORY motor
  TOOLCHAIN_FILE ${ROSSERIAL_TIVAC_TOOLCHAIN}
)
rosserial_add_client_target(motor motor.axf ALL)
rosserial_add_client_target(motor flash)
rosserial_add_client_target(motor size)
rosserial_add_client_target(motor dump)

# drive encoders
rosserial_configure_client(
  DIRECTORY drive_encoder
  TOOLCHAIN_FILE ${ROSSERIAL_TIVAC_TOOLCHAIN}
)
rosserial_add_client_target(drive_encoder drive_encoder.axf ALL)
rosserial_add_client_target(drive_encoder flash)
rosserial_add_client_target(drive_encoder size)
rosserial_add_client_target(drive_encoder dump)
