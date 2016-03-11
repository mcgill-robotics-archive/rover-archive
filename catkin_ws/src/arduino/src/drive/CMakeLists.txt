cmake_minimum_required(VERSION 2.8.3)

include_directories(${ROS_LIB_DIR})

# Remove this if using an Arduino without native USB (eg, other than Leonardo)
# add_definitions(-DUSB_CON)

add_subdirectory(MotorController)

set(SOURCES
        main.cpp
        ../common/ram.cpp
        ${ROS_LIB_DIR}/time.cpp
        SteeringWheel.cpp
        Wheel.cpp
        )

set(HEADERS
        ../common/ram.h
        SteeringWheel.h
        Wheel.h
        pins.h
        )

generate_arduino_firmware(drive
    SRCS ${SOURCES}
    HDRS ${HEADERS}
    LIBS MotorControllerLib
    BOARD mega2560
    PORT /dev/ttyACM0
)
