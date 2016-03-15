cmake_minimum_required(VERSION 2.8.3)

include_directories(${ROS_LIB_DIR})

set(MC_SOURCES
        DRV8308.cpp
        MAXON.cpp
        MotorConfig.cpp
        MotorController.cpp)

set(MC_HEADERS
        DRV8308.h
        MAXON.h
        MotorConfig.h
        MotorController.h)

generate_arduino_library(MotorControllerLib
        SRCS ${MC_SOURCES}
        HDRS ${MC_HEADERS}
        BOARD mega2560)