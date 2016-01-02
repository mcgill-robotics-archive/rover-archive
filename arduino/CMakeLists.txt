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

# add the project directory into build
add_subdirectory(RS232-router-test)
