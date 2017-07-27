# Locates the RIMSTREAMER header files and libraries.
#
# The following variables are set if RIMSTREAMER is found.
#   RIMSTREAMER_FOUND - True when the RIMSTREAMER include directory is found.
#   RIMSTREAMER_INCLUDE_DIRS - Paths to the RIMSTREAMER header files.
#   RIMSTREAMER_LIBRARIES - List of all the libraries from the requested components.
#   RIMSTREAMER_DEFINITIONS - Compiler switches required for using RIMSTREAMER
#
# Accepts the following variables as input:
#   RIMSTREAMER_ROOT - Root directory of the RIMSTREAMER install prefix
#               (as a CMake or environment variable)
#
# -----------------------------------------------------------------
# Usage:
#
#   find_package(RIMSTREAMER)
#   ...
#   include_directories(${RIMSTREAMER_INCLUDE_DIRS})
#   link_directories(${RIMSTREAMER_LIBRARIES})
# -----------------------------------------------------------------
#


# Use pkg-config unless you are on Windows
if (NOT WIN32)
  find_package(PkgConfig)
  pkg_check_modules(PC_RIMSTREAMER QUIET rimstreamer>=1.0.0)
  set(RIMSTREAMER_DEFINITIONS ${PC_RIMSTREAMER_CFLAGS_OTHER})
endif()


# Find the header files
find_path(RIMSTREAMER_INCLUDE_DIR
          rimstreamer/VideoFeed.h
          HINTS ${PC_RIMSTREAMER_INCLUDEDIR} ${PC_RIMSTREAMER_INCLUDE_DIRS}
          PATHS ${RIMSTREAMER_ROOT}/include $ENV{RIMSTREAMER_ROOT}/include)
set(RIMSTREAMER_INCLUDE_DIRS ${RIMSTREAMER_INCLUDE_DIR})


# Find QtGStreamer and add it to the dependencies
set(Qt5GStreamer_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../qt-gstreamer_lib/lib/cmake/Qt5GStreamer")

#find_package(Qt5Widgets REQUIRED)
find_package(Qt5GStreamer REQUIRED)

if (UNIX)

  # Find the static library
  find_library(RIMSTREAMER_LIBRARY
    NAMES rimstreamer
    PATHS ${RIMSTREAMER_ROOT}/lib $ENV{RIMSTREAMER_ROOT}/lib)
  if (RIMSTREAMER_LIBRARY)
    set(RIMSTREAMER_FOUND true)
    list(APPEND RIMSTREAMER_LIBRARIES "${RIMSTREAMER_LIBRARY}")
    list(APPEND RIMSTREAMER_LIBRARIES "${QTGSTREAMER_LIBRARIES}"
                                      "${QTGSTREAMER_UI_LIBRARIES}"
                                      "${QTGSTREAMER_UTILS_LIBRARIES}")
  endif ()

endif()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(RIMSTREAMER
                                  REQUIRED_VARS RIMSTREAMER_INCLUDE_DIRS RIMSTREAMER_LIBRARIES)


# Mark all the variables as advanced
list(APPEND _RIMSTREAMER_COMPONENT_LIBRARY_VARS RIMSTREAMER_LIBRARY_RELEASE
                                                RIMSTREAMER_LIBRARY_DEBUG
                                                RIMSTREAMER_LIBRARY)
mark_as_advanced(RIMSTREAMER_INCLUDE_DIR
                 ${_RIMSTREAMER_COMPONENT_LIBRARY_VARS})

