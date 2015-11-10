cmake_minimum_required(VERSION 2.8.3)

generate_arduino_firmware(rs232
SRCS main.cpp CustomSerial.cpp
HDRS CustomSerial.h
BOARD uno
PORT /dev/tty.usbserial-DA00WSQJ)
