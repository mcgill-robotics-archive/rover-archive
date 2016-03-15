include_directories(${ROS_LIB_DIR})
include_directories(../MotorController)
include_directories(../utilities)

# Remove this if using an Arduino without native USB (eg, other than Leonardo)
# add_definitions(-DUSB_CON)

set(SOURCES
        arm.cpp
        ../MotorController/src/Pololu.cpp
        ${ROS_LIB_DIR}/time.cpp
        Encoder.cpp
        PitchRollCompute.cpp
        TransformFrame/TransformSender.cpp
        TransformFrame/TransformConfig.cpp
        )

set(HEADERS
        arm.h
        ../MotorController/src/Pololu.h
        Encoder.h
        PitchRollCompute.h
        TransformFrame/TransformSender.h
        TransformFrame/TransformConfig.h
        pins_arm.h)

generate_arduino_firmware(arm
    SRCS ${SOURCES}
    HDRS ${HEADERS}
    LIBS MotorControllerLib ram pid
    BOARD mega2560
    PORT /dev/mega_arm)
