cmake_minimum_required(VERSION 2.8.3)

generate_arduino_library(baro_driver
        SRCS Adafruit_MPL3115A2.cpp
        HDRS Adafruit_MPL3115A2.h
        BOARD uno)
