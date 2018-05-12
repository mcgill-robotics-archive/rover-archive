cmake_minimum_required(VERSION 2.8.3)

generate_arduino_library(encoder
        SRCS Encoder.cpp
        HDRS Encoder.h
        BOARD micro)
