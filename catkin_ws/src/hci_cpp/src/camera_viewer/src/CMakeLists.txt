include_directories(${CMAKE_CURRENT_SOURCE_DIR})

file(GLOB SOURCES *.cpp)

add_executable(${PROJECT_NAME}/camera_viewer ${SOURCES})

target_link_libraries(${PROJECT_NAME}/camera_viewer
        Qt5::Widgets
        qFlightInstruments
        ${RIMSTREAMER_LIBRARIES}
        ${QTGSTREAMER_LIBRARIES}
        ${QTGSTREAMER_UI_LIBRARIES})
