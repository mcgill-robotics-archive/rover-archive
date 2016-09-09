cmake_minimum_required(VERSION 2.8.3)

include_directories(${ROS_LIB_DIR})
include_directories(../utilities)
include_directories(../MotorController/include)


# Remove this if using an Arduino without native USB (eg, other than Leonardo)
# add_definitions(-DUSB_CON)

file(GLOB SCIENCE_SRCS [^.]*.cpp)
file(GLOB SCIENCE_HDRS [^.]*.h)

generate_arduino_firmware(science
        SRCS ${SCIENCE_SRCS} ${SCIENCE_HDRS} ${ROS_LIB_DIR}/time.cpp
        BOARD mega2560
        LIBS MotorControllerLib baro_driver hx711
        PORT /dev/ttyACM0
        )

