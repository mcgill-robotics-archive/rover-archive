cmake_minimum_required(VERSION 2.8.3)

include_directories(${ROS_LIB_DIR})

generate_arduino_library(pid
        SRCS PID_v1.cpp
        HDRS PID_v1.h
        BOARD mega2560)
