# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# Instruct CMake to run moc automatically when needed.
set(CMAKE_AUTOMOC ON)

file(GLOB HCI_SOURCES [^.]*.cpp)
file(GLOB HCI_HEADERS [^.]*.h)

file(GLOB VIEW_SOURCES view/[^.]*.cpp)
file(GLOB VIEW_HEADERS view/[^.]*.h)

file(GLOB MODEL_SOURCES model/[^.]*.cpp)
file(GLOB MODEL_HEADERS model/[^.]*.h)

file(GLOB CONTROLLER_SOURCES controller/[^.]*.cpp)
file(GLOB CONTROLLER_HEADERS controller/[^.]*.h)

file(GLOB QRC_SOURCES resources/[^.]*.qrc)
qt5_add_resources(GENERATED_RESOURCES ${QRC_SOURCES})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")

add_executable(hci
        ${HCI_SOURCES}
        ${HCI_HEADERS}
        ${VIEW_SOURCES}
        ${VIEW_HEADERS}
        ${MODEL_SOURCES}
        ${MODEL_HEADERS}
        ${CONTROLLER_SOURCES}
        ${CONTROLLER_HEADERS}
        ${GENERATED_RESOURCES})

target_link_libraries(hci
        Qt5::Widgets
        ${RIMSTREAMER_LIBRARIES}
        ${catkin_LIBRARIES}
        qFlightInstruments)
