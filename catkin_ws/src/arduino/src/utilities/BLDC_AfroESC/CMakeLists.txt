cmake_minimum_required(VERSION 2.8.3)

generate_arduino_library(bldc_afroesc
        SRCS BLDC_AfroESC.cpp
        HDRS BLDC_AfroESC.h
        BOARD micro)
