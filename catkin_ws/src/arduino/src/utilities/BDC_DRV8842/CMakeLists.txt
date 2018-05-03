cmake_minimum_required(VERSION 2.8.3)

generate_arduino_library(bdc_drv8842
        SRCS BDC_DRV8842.cpp
        HDRS BDC_DRV8842.h
        BOARD micro)
