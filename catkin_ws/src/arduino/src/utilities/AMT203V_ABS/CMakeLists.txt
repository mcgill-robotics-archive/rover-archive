cmake_minimum_required(VERSION 2.8.3)

generate_arduino_library(amt203v_abs
        SRCS AMT203-V_ABS.cpp
        HDRS AMT203-V_ABS.h
        BOARD micro)
