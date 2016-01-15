set(CMAKE_TOOLCHAIN_FILE cmake/ArduinoToolchain.cmake) # Arduino Toolchain

cmake_minimum_required(VERSION 2.8)
#====================================================================#
#  Setup Project                                                     #
#====================================================================#
project(Rover-Arduino C CXX)

print_board_list()
print_programmer_list()

# add libraries to project
link_directories(${ARDUINO_SDK}/libraries)

set(ROS_LIB_DIR ${CMAKE_CURRENT_LIST_DIR}/../catkin_ws/build/arduino/ros_lib)

# add the project directory into build
add_subdirectory(drive)
add_subdirectory(power)
add_subdirectory(arm)
