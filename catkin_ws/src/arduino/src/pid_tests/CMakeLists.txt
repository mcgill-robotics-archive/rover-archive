include_directories(${ROS_LIB_DIR})
include_directories(../utilities)

# Remove this if using an Arduino without native USB (eg, other than Leonardo)
# add_definitions(-DUSB_CON)

set(SOURCES
        main.cpp
        ${ROS_LIB_DIR}/time.cpp
        )


generate_arduino_firmware(pid_tests
    SRCS ${SOURCES}
    LIBS QuadEncoder
    BOARD mega2560
    PORT /dev/mega_science
)
