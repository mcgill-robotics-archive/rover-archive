cmake_minimum_required(VERSION 2.8.3)

include_directories(${ROS_LIB_DIR})


# Remove this if using an Arduino without native USB (eg, other than Leonardo)
# add_definitions(-DUSB_CON)

set(SOURCES
        arm.cpp
        ../common/ram.cpp
        ${ROS_LIB_DIR}/time.cpp
        Encoder.cpp
        PitchRollCompute.cpp
        TransformFrame/TransformSender.cpp
        TransformFrame/TransformConfig.cpp
        PID_v1/PID_v1.cpp)

set(HEADERS
        arm.h
        ../common/ram.h
        Encoder.h
        PitchRollCompute.h
        TransformFrame/TransformSender.h
        TransformFrame/TransformConfig.h
        PID_v1/PID_v1.h)

generate_arduino_firmware(arm
    SRCS ${SOURCES}
    HDRS ${HEADERS}
    BOARD mega2560
    PORT /dev/mega_test
)
