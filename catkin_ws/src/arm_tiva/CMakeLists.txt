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
rosserial_configure_client(
 DIRECTORY encoder_magnetic
 TOOLCHAIN_FILE ${ROSSERIAL_TIVAC_TOOLCHAIN}
)
rosserial_add_client_target(encoder_magnetic encoder_magnetic.axf ALL)
rosserial_add_client_target(encoder_magnetic flash)
rosserial_add_client_target(encoder_magnetic size)
rosserial_add_client_target(encoder_magnetic dump)

# drive encoders left
rosserial_configure_client(
  DIRECTORY drive_encoder_left
  TOOLCHAIN_FILE ${ROSSERIAL_TIVAC_TOOLCHAIN}
)
rosserial_add_client_target(drive_encoder_left drive_encoder_left.axf ALL)
rosserial_add_client_target(drive_encoder_left flash)
rosserial_add_client_target(drive_encoder_left size)
rosserial_add_client_target(drive_encoder_left dump)

# drive encoders right
rosserial_configure_client(
  DIRECTORY drive_encoder_right
  TOOLCHAIN_FILE ${ROSSERIAL_TIVAC_TOOLCHAIN}
)
rosserial_add_client_target(drive_encoder_right drive_encoder_right.axf ALL)
rosserial_add_client_target(drive_encoder_right flash)
rosserial_add_client_target(drive_encoder_right size)
rosserial_add_client_target(drive_encoder_right dump)
