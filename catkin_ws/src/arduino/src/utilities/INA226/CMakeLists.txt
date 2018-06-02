cmake_minimum_required(VERSION 2.8.3)
add_definitions(-std=c++11)

generate_arduino_library(ina226
        SRCS INA226.cpp
        HDRS INA226.h
        BOARD mega2560)
