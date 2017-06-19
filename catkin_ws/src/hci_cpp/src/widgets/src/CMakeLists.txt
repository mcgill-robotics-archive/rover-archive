set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# Instruct CMake to run moc automatically when needed.
set(CMAKE_AUTOMOC ON)

file(GLOB qFlightInstruments_SOURCES [^.]*.cpp)
file(GLOB qFlightInstruments_PUBLIC_HEADERS
        ${CMAKE_CURRENT_SOURCE_DIR}/../includes/qFlightInstruments/[^.]*.h)

add_library(qFlightInstruments
        ${qFlightInstruments_PUBLIC_HEADERS}
        ${qFlightInstruments_SOURCES})

target_link_libraries(qFlightInstruments Qt5::Widgets)
