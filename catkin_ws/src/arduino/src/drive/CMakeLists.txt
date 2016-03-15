cmake_minimum_required(VERSION 2.8.3)

include_directories(${ROS_LIB_DIR})
include_directories(../utilities)
include_directories(../MotorController/include)

# Remove this if using an Arduino without native USB (eg, other than Leonardo)
# add_definitions(-DUSB_CON)

set(SOURCES
        main.cpp
        ${ROS_LIB_DIR}/time.cpp
        SteeringWheel.cpp
        Wheel.cpp
        )

set(HEADERS
        SteeringWheel.h
        Wheel.h
        pins_drive.h
        )

generate_arduino_firmware(drive
    SRCS ${SOURCES}
    HDRS ${HEADERS}
    LIBS MotorControllerLib ram
    BOARD mega2560
    PORT /dev/ttyACM0
)
