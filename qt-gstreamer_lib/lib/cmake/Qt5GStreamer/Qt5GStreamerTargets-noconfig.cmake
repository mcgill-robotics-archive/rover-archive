#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "Qt5GLib" for configuration ""
set_property(TARGET Qt5GLib APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(Qt5GLib PROPERTIES
  IMPORTED_LINK_INTERFACE_LIBRARIES_NOCONFIG "Qt5::Core"
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libQt5GLib-2.0.so.1.2.0"
  IMPORTED_SONAME_NOCONFIG "libQt5GLib-2.0.so.0"
  )

list(APPEND _IMPORT_CHECK_TARGETS Qt5GLib )
list(APPEND _IMPORT_CHECK_FILES_FOR_Qt5GLib "${_IMPORT_PREFIX}/lib/libQt5GLib-2.0.so.1.2.0" )

# Import target "Qt5GStreamer" for configuration ""
set_property(TARGET Qt5GStreamer APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(Qt5GStreamer PROPERTIES
  IMPORTED_LINK_INTERFACE_LIBRARIES_NOCONFIG "Qt5GLib;Qt5::Core"
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libQt5GStreamer-1.0.so.1.2.0"
  IMPORTED_SONAME_NOCONFIG "libQt5GStreamer-1.0.so.0"
  )

list(APPEND _IMPORT_CHECK_TARGETS Qt5GStreamer )
list(APPEND _IMPORT_CHECK_FILES_FOR_Qt5GStreamer "${_IMPORT_PREFIX}/lib/libQt5GStreamer-1.0.so.1.2.0" )

# Import target "Qt5GStreamerQuick" for configuration ""
set_property(TARGET Qt5GStreamerQuick APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(Qt5GStreamerQuick PROPERTIES
  IMPORTED_LINK_DEPENDENT_LIBRARIES_NOCONFIG "Qt5::Core"
  IMPORTED_LINK_INTERFACE_LIBRARIES_NOCONFIG "Qt5GStreamer;Qt5::Quick"
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libQt5GStreamerQuick-1.0.so.1.2.0"
  IMPORTED_SONAME_NOCONFIG "libQt5GStreamerQuick-1.0.so.0"
  )

list(APPEND _IMPORT_CHECK_TARGETS Qt5GStreamerQuick )
list(APPEND _IMPORT_CHECK_FILES_FOR_Qt5GStreamerQuick "${_IMPORT_PREFIX}/lib/libQt5GStreamerQuick-1.0.so.1.2.0" )

# Import target "Qt5GStreamerUi" for configuration ""
set_property(TARGET Qt5GStreamerUi APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(Qt5GStreamerUi PROPERTIES
  IMPORTED_LINK_DEPENDENT_LIBRARIES_NOCONFIG "Qt5::Core;Qt5::OpenGL"
  IMPORTED_LINK_INTERFACE_LIBRARIES_NOCONFIG "Qt5GStreamer;Qt5::Widgets;Qt5::Gui"
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libQt5GStreamerUi-1.0.so.1.2.0"
  IMPORTED_SONAME_NOCONFIG "libQt5GStreamerUi-1.0.so.0"
  )

list(APPEND _IMPORT_CHECK_TARGETS Qt5GStreamerUi )
list(APPEND _IMPORT_CHECK_FILES_FOR_Qt5GStreamerUi "${_IMPORT_PREFIX}/lib/libQt5GStreamerUi-1.0.so.1.2.0" )

# Import target "Qt5GStreamerUtils" for configuration ""
set_property(TARGET Qt5GStreamerUtils APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(Qt5GStreamerUtils PROPERTIES
  IMPORTED_LINK_DEPENDENT_LIBRARIES_NOCONFIG "Qt5::Core"
  IMPORTED_LINK_INTERFACE_LIBRARIES_NOCONFIG "Qt5GStreamer"
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libQt5GStreamerUtils-1.0.so.1.2.0"
  IMPORTED_SONAME_NOCONFIG "libQt5GStreamerUtils-1.0.so.0"
  )

list(APPEND _IMPORT_CHECK_TARGETS Qt5GStreamerUtils )
list(APPEND _IMPORT_CHECK_FILES_FOR_Qt5GStreamerUtils "${_IMPORT_PREFIX}/lib/libQt5GStreamerUtils-1.0.so.1.2.0" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
