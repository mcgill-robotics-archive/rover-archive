include_directories(${ROS_LIB_DIR})
include_directories(../utilities)

# Remove this if using an Arduino without native USB (eg, other than Leonardo)
add_definitions(-DUSB_CON)

set(SOURCES
        arm.cpp
        ${ROS_LIB_DIR}/time.cpp
        )

set(HEADERS
        )

generate_arduino_firmware(drive
    SRCS ${SOURCES}
    HDRS ${HEADERS}
    LIBS amt203v_abs bdc_drv8842 ina226
    BOARD micro
    PORT /dev/ttyACM0
)
