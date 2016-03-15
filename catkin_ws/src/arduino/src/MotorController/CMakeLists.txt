include_directories(${ROS_LIB_DIR})

set(MC_SOURCES
        src/DRV8308.cpp
        src/MAXON.cpp
        src/Pololu.cpp
        src/MotorConfig.cpp
        src/MotorController.cpp)

set(MC_HEADERS
        src/DRV8308.h
        src/MAXON.h
        src/Pololu.h
        include/MotorConfig.h
        include/MotorController.h)

generate_arduino_library(MotorControllerLib
        SRCS ${MC_SOURCES}
        HDRS ${MC_HEADERS}
        BOARD mega2560)