cmake_minimum_required(VERSION 2.8.3)

include_directories(${ROS_LIB_DIR})

# Remove this if using an Arduino without native USB (eg, other than Leonardo)
# add_definitions(-DUSB_CON)

set(SOURCES
        main.cpp
        ${ROS_LIB_DIR}/time.cpp
        SteeringWheel.cpp
        Wheel.cpp
        MotorController/MAXON.cpp
        MotorController/DRV8308.cpp
        MotorController/MotorConfig.cpp
        MotorController/MotorController.cpp
        )


set(HEADERS
        SteeringWheel.h
        Wheel.h
        MotorController/MAXON.h
        MotorController/DRV8308.h
        MotorController/MotorConfig.h
        MotorController/MotorController.h
        )
generate_arduino_firmware(drive
    SRCS ${SOURCES}
    HDRS ${HEADERS}
    BOARD mega2560
    PORT /dev/mega_drive
)
