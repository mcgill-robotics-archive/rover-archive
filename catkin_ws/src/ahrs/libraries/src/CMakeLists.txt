add_library(common STATIC
    PeriodicThread.cpp
    ../include/common/PeriodicThread.h
)
target_link_libraries(common
     ${Boost_THREAD_LIBRARIES}
     ${Boost_SYSTEM_LIBRARIES}
)


file(GLOB AHRS_SOURCES [^.]*.cpp)
file(GLOB AHRS_PRIVATE_HEADERS [^.]*.h)
file(GLOB AHRS_PUBLIC_HEADERS ../include/ahrs/[^.]*.h)

add_library(ahrs_lib STATIC
            ${AHRS_SOURCES}
            ${AHRS_PRIVATE_HEADERS}
            ${AHRS_PUBLIC_HEADERS}
            ${Boost_THREAD_LIBRARIES}
            ${Boost_SYSTEM_LIBRARIES})

target_link_libraries(ahrs_lib sbgCom common)
