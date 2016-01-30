cmake_minimum_required(VERSION 2.8.3)

include_directories(${ROS_LIB_DIR})


# Remove this if using an Arduino without native USB (eg, other than Leonardo)
# add_definitions(-DUSB_CON)

set(SOURCES main.cpp ${ROS_LIB_DIR}/time.cpp Encoder.cpp PitchRollCompute.cpp)
set(HEADERS Encoder.h PitchRollCompute.h)

generate_arduino_firmware(arm
    SRCS ${SOURCES}
    HDRS ${HEADERS}
    BOARD mega2560
    LIBS pid
    PORT /dev/mega_arm
)
