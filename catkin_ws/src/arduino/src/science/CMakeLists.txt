include_directories(${ROS_LIB_DIR})
include_directories(../utilities)

# Remove this if using an Arduino without native USB (eg, other than Leonardo)
# add_definitions(-DUSB_CON)

set(SOURCES
        science_controller.cpp
        )

set(HEADERS
        )

generate_arduino_firmware(science
    SRCS ${SOURCES}
    HDRS ${HEADERS}
    LIBS amt203v_inc bdv_drv8842 ina226
    BOARD mega2560
    PORT /dev/mega_science
)
