add_library(common STATIC
    ../include/common/PeriodicThread.h
    PeriodicThread.cpp
)
target_link_libraries(common
     ${Boost_THREAD_LIBRARIES}
     ${Boost_SYSTEM_LIBRARIES}
)


file(GLOB AHRS_SOURCES [^.]*.cpp)
file(GLOB AHRS_PRIVATE_HEADERS [^.]*.h)
file(GLOB AHRS_PUBLIC_HEADERS ../include/${PROJECT_NAME}/[^.]*.h)

add_library(${PROJECT_NAME}_lib STATIC
            ${AHRS_SOURCES}
            ${AHRS_PRIVATE_HEADERS}
            ${AHRS_PUBLIC_HEADERS}
            ${Boost_THREAD_LIBRARIES}
            ${Boost_SYSTEM_LIBRARIES})

target_link_libraries(${PROJECT_NAME}_lib sbgCom common)

install(TARGETS ${PROJECT_NAME}_lib common
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
