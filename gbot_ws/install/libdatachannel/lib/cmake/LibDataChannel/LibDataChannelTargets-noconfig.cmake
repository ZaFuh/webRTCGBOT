#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "LibDataChannel::LibDataChannel" for configuration ""
set_property(TARGET LibDataChannel::LibDataChannel APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(LibDataChannel::LibDataChannel PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libdatachannel.so.0.22.5"
  IMPORTED_SONAME_NOCONFIG "libdatachannel.so.0.22"
  )

list(APPEND _IMPORT_CHECK_TARGETS LibDataChannel::LibDataChannel )
list(APPEND _IMPORT_CHECK_FILES_FOR_LibDataChannel::LibDataChannel "${_IMPORT_PREFIX}/lib/libdatachannel.so.0.22.5" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
