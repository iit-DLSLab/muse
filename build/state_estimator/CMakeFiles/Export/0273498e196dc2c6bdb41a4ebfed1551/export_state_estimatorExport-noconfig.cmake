#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "state_estimator::state_estimator_plugins" for configuration ""
set_property(TARGET state_estimator::state_estimator_plugins APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(state_estimator::state_estimator_plugins PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libstate_estimator_plugins.so"
  IMPORTED_SONAME_NOCONFIG "libstate_estimator_plugins.so"
  )

list(APPEND _cmake_import_check_targets state_estimator::state_estimator_plugins )
list(APPEND _cmake_import_check_files_for_state_estimator::state_estimator_plugins "${_IMPORT_PREFIX}/lib/libstate_estimator_plugins.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
