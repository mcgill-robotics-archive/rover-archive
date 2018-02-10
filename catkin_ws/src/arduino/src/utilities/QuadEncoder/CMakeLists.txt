include_directories(${ROS_LIB_DIR})

generate_arduino_library(QuadEncoder
        SRCS Encoder.cpp
        HDRS Encoder.h
        BOARD mega2560)
