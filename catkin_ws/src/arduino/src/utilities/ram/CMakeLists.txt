include_directories(${ROS_LIB_DIR})
generate_arduino_library(ram
        SRCS ram.cpp
        HDRS ram.h
        BOARD mega2560)
