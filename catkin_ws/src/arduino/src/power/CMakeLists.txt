include_directories(${ROS_LIB_DIR})
include_directories(../utilities)

# Remove this if using an Arduino without native USB (eg, other than Leonardo)
add_definitions(-DUSB_CON)

set(SOURCES
        power.cpp
        ${ROS_LIB_DIR}/time.cpp
        )

set(HEADERS
        )

generate_arduino_firmware(drive
    SRCS ${SOURCES}
    HDRS ${HEADERS}
    LIBS ina3221
    BOARD micro
    PORT /dev/ttyACM0
)
