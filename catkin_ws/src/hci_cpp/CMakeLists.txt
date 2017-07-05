cmake_minimum_required(VERSION 2.8.3)
project(hci_cpp)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake-modules")
include(Findrimstreamer)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        sensor_msgs
        std_msgs
        drive_control
        rover_common
        arm_control
        )

include_directories(
        ${catkin_INCLUDE_DIRS}
)

catkin_package(CATKIN_DEPENDS roscpp sensor_msgs std_msgs drive_control rover_common arm_control)

find_package(Qt5Widgets REQUIRED)
find_package(Qt5GStreamer 1.2.0 REQUIRED)
find_package(rimstreamer REQUIRED)

include_directories(${QTGSTREAMER_INCLUDES})
include_directories(${RIMSTREAMER_INCLUDE_DIRS})

add_subdirectory(src)
