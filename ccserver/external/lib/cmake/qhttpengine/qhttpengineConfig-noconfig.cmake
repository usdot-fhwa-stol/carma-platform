#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "qhttpengine" for configuration ""
set_property(TARGET qhttpengine APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(qhttpengine PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libqhttpengine.so.1.0.1"
  IMPORTED_SONAME_NOCONFIG "libqhttpengine.so.1"
  )

list(APPEND _IMPORT_CHECK_TARGETS qhttpengine )
list(APPEND _IMPORT_CHECK_FILES_FOR_qhttpengine "${_IMPORT_PREFIX}/lib/libqhttpengine.so.1.0.1" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
