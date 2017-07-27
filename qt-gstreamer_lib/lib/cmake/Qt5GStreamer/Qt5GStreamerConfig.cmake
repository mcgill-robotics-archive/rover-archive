set(QTGSTREAMER_VERSION 1.2.0)


####### Expanded from @PACKAGE_INIT@ by configure_package_config_file() #######
####### Any changes to this file will be overwritten by the next CMake run ####
####### The input file was QtGStreamerConfig.cmake.in                            ########

get_filename_component(PACKAGE_PREFIX_DIR "${CMAKE_CURRENT_LIST_DIR}/../../../" ABSOLUTE)

macro(set_and_check _var _file)
  set(${_var} "${_file}")
  if(NOT EXISTS "${_file}")
    message(FATAL_ERROR "File or directory ${_file} referenced by variable ${_var} does not exist !")
  endif()
endmacro()

####################################################################################

get_filename_component(_QTGSTREAMER_CONFIG_DIR "${CMAKE_CURRENT_LIST_FILE}" PATH)

# include the generated QtGStreamerTargets.cmake from the same directory
find_file(_QTGSTREAMER_TARGETS_FILE Qt5GStreamerTargets.cmake
          PATHS ${_QTGSTREAMER_CONFIG_DIR} NO_DEFAULT_PATH)
include(${_QTGSTREAMER_TARGETS_FILE})
unset(_QTGSTREAMER_TARGETS_FILE)

set(Qt5GStreamer_FOUND TRUE)
set(QTGLIB_LIBRARY Qt5GLib)
set(QTGSTREAMER_LIBRARY Qt5GStreamer)
set(QTGSTREAMER_QUICK_LIBRARY Qt5GStreamerQuick)
set(QTGSTREAMER_UI_LIBRARY Qt5GStreamerUi)
set(QTGSTREAMER_UTILS_LIBRARY Qt5GStreamerUtils)
set_and_check(QTGSTREAMER_INCLUDE_DIR ${PACKAGE_PREFIX_DIR}/include/Qt5GStreamer)

if (Qt5GStreamer_FIND_QUIET)
    set(_QTGSTREAMER_FIND_DEPS_ARGS QUIET)
endif()

# Find dependencies, if not already found
if ("Qt5GStreamer" STREQUAL "Qt5GStreamer")
    if (NOT DEFINED Qt5Core_INCLUDE_DIRS)
        if (NOT Qt5GStreamer_FIND_QUIET)
            message(STATUS "Qt5 hasn't been found yet. Looking...")
        endif()

        find_package(Qt5Core ${_QTGSTREAMER_FIND_DEPS_ARGS})

        # import targets for linking to QtGStreamerUi, but don't fail
        # if they are not found. One may only want QtGStreamer (no Ui).
        find_package(Qt5Widgets QUIET)

        if (NOT Qt5Core_FOUND)
            set (Qt5GStreamer_FOUND FALSE)
        endif()
    endif()
else()
    if (NOT DEFINED QT_INCLUDE_DIR)
        if (NOT Qt5GStreamer_FIND_QUIET)
            message(STATUS "Qt4 hasn't been found yet. Looking...")
        endif()

        find_package(Qt4 COMPONENTS QtCore ${_QTGSTREAMER_FIND_DEPS_ARGS})

        if (NOT Qt4_FOUND)
            set (Qt5GStreamer_FOUND FALSE)
        endif()
    endif()
endif()

if (NOT DEFINED Boost_INCLUDE_DIRS)
    if (NOT Qt5GStreamer_FIND_QUIET)
        message(STATUS "Boost hasn't been found yet. Looking...")
    endif()

    find_package(Boost ${_QTGSTREAMER_FIND_DEPS_ARGS})

    if (NOT Boost_FOUND)
        set (Qt5GStreamer_FOUND FALSE)
    endif()
endif()

unset(_QTGSTREAMER_FIND_DEPS_ARGS)

# include QtGStreamerConfigCommon.cmake from the same directory
find_file(_QTGSTREAMER_CONFIG_COMMON_FILE QtGStreamerConfigCommon.cmake
          PATHS ${_QTGSTREAMER_CONFIG_DIR} NO_DEFAULT_PATH)
include(${_QTGSTREAMER_CONFIG_COMMON_FILE})
unset(_QTGSTREAMER_CONFIG_COMMON_FILE)

# compatibility variable
set(QTGSTREAMER_FOUND ${Qt5GStreamer_FOUND})
