cmake_minimum_required(VERSION 2.8.3)
project(arm_tiva)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  rosserial_client
  rosserial_tivac
  std_msgs
)

# add_message_files(
#   # FILES Buttons.msg
# )
# add_service_files(
#   # FILES ColorRGBA.srv
# )

# generate_messages(
#   DEPENDENCIES std_msgs
# )

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs
)

rosserial_generate_ros_lib(
  PACKAGE rosserial_tivac
  SCRIPT make_libraries_tiva
)
# Because we need to generate our custom messages before exporting our rosserial ros_lib
# add_dependencies(${PROJECT_NAME}_ros_lib ${${PROJECT_NAME}_EXPORTED_TARGETS})
# add_dependencies(${PROJECT_NAME}_ros_lib ${catkin_EXPORTED_TARGETS})

# encoder
rosserial_configure_client(
  DIRECTORY encoder
  TOOLCHAIN_FILE ${ROSSERIAL_TIVAC_TOOLCHAIN}
)
rosserial_add_client_target(encoder encoder.axf ALL)
rosserial_add_client_target(encoder flash)
rosserial_add_client_target(encoder size)
rosserial_add_client_target(encoder dump)
