cmake_minimum_required(VERSION 2.8.3)

generate_arduino_library(bmp085_driver
    SRCS Adafruit_BMP085.cpp
    HDRS Adafruit_BMP085.h
        BOARD uno)
