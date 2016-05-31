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

if(OSX)
    MESSAGE("APPLE SYSTEM")
    set(ROS_LIB_DIR ${CMAKE_CURRENT_LIST_DIR}/../catkin_ws/build_isolated/arduino/ros_lib)
else()
    set(ROS_LIB_DIR ${CMAKE_CURRENT_LIST_DIR}/../catkin_ws/build/arduino/ros_lib)
endif()

# add the project directory into build
add_subdirectory(src)
