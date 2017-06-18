message("${CMAKE_CURRENT_SOURCE_DIR}")

# Instruct CMake to run moc automatically when needed.
set(CMAKE_AUTOMOC ON)

# We need add -DQT_WIDGETS_LIB when using QtWidgets in Qt 5.
add_definitions(${Qt5Widgets_DEFINITIONS})

if(UNIX)
    # Executables fail to build with Qt 5 in the default configuration
    # without -fPIE. We add that here.
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${Qt5Widgets_EXECUTABLE_COMPILE_FLAGS}")
endif()

add_subdirectory(src)
