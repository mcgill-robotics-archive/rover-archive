cmake_minimum_required(VERSION 2.8.3)

generate_arduino_library(amt203v_inc
        SRCS AMT203-V_INC.cpp
        HDRS AMT203-V_INC.h
        BOARD micro)
