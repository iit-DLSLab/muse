# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_iit_commons_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED iit_commons_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(iit_commons_FOUND FALSE)
  elseif(NOT iit_commons_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(iit_commons_FOUND FALSE)
  endif()
  return()
endif()
set(_iit_commons_CONFIG_INCLUDED TRUE)

# output package information
if(NOT iit_commons_FIND_QUIETLY)
  message(STATUS "Found iit_commons: 1.0.0 (${iit_commons_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'iit_commons' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT iit_commons_DEPRECATED_QUIET)
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(iit_commons_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "ament_cmake_export_include_directories-extras.cmake;ament_cmake_export_libraries-extras.cmake;ament_cmake_export_dependencies-extras.cmake")
foreach(_extra ${_extras})
  include("${iit_commons_DIR}/${_extra}")
endforeach()
