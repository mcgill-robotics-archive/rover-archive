# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# Instruct CMake to run moc automatically when needed.
set(CMAKE_AUTOMOC ON)

file(GLOB HCI_SOURCES [^.]*.cpp)
file(GLOB HCI_HEADERS [^.]*.h)

file(GLOB VIEW_SOURCES view/[^.]*.cpp)
file(GLOB VIEW_HEADERS view/[^.]*.h)



add_executable(hci ${HCI_SOURCES} ${HCI_HEADERS} ${VIEW_SOURCES} ${VIEW_HEADERS})
target_link_libraries(hci
        Qt5::Widgets
        ${RIMSTREAMER_LIBRARIES}
        qFlightInstruments)