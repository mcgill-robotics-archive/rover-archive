cmake_minimum_required(VERSION 2.8.3)
project(arduino)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  rosserial_arduino
  rosserial_client
  std_msgs
  ahrs
)

set(ARDUINO_SDK_PATH "/home/david/arduino-1.6.5")

rosserial_generate_ros_lib(
    PACKAGE rosserial_arduino
    SCRIPT make_libraries.py
)

rosserial_configure_client(
    DIRECTORY drive
    TOOLCHAIN_FILE ${ROSSERIAL_ARDUINO_TOOLCHAIN}
)


rosserial_configure_client(
    DIRECTORY RS232-router-test 
    TOOLCHAIN_FILE ${ROSSERIAL_ARDUINO_TOOLCHAIN}
)

rosserial_add_client_target(drive drive ALL)
rosserial_add_client_target(drive drive-upload)

rosserial_add_client_target(RS232-router-test rs232 ALL)
rosserial_add_client_target(RS232-router-test rs232-upload)
rosserial_add_client_target(RS232-router-test rs232-size)

