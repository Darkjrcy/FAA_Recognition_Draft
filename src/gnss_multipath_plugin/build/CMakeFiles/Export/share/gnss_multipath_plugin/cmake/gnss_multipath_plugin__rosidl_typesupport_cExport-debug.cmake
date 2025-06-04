#----------------------------------------------------------------
# Generated CMake target import file for configuration "Debug".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "gnss_multipath_plugin::gnss_multipath_plugin__rosidl_typesupport_c" for configuration "Debug"
set_property(TARGET gnss_multipath_plugin::gnss_multipath_plugin__rosidl_typesupport_c APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_target_properties(gnss_multipath_plugin::gnss_multipath_plugin__rosidl_typesupport_c PROPERTIES
  IMPORTED_LOCATION_DEBUG "${_IMPORT_PREFIX}/lib/libgnss_multipath_plugin__rosidl_typesupport_c.so"
  IMPORTED_SONAME_DEBUG "libgnss_multipath_plugin__rosidl_typesupport_c.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS gnss_multipath_plugin::gnss_multipath_plugin__rosidl_typesupport_c )
list(APPEND _IMPORT_CHECK_FILES_FOR_gnss_multipath_plugin::gnss_multipath_plugin__rosidl_typesupport_c "${_IMPORT_PREFIX}/lib/libgnss_multipath_plugin__rosidl_typesupport_c.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
