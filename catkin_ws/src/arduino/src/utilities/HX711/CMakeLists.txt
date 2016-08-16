cmake_minimum_required(VERSION 2.8.3)

generate_arduino_library(hx711
        SRCS HX711.cpp
        HDRS HX711.h
        BOARD uno)
