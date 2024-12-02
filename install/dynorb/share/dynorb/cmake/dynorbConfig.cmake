# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_dynorb_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED dynorb_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(dynorb_FOUND FALSE)
  elseif(NOT dynorb_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(dynorb_FOUND FALSE)
  endif()
  return()
endif()
set(_dynorb_CONFIG_INCLUDED TRUE)

# output package information
if(NOT dynorb_FIND_QUIETLY)
  message(STATUS "Found dynorb: 0.0.0 (${dynorb_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'dynorb' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${dynorb_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(dynorb_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${dynorb_DIR}/${_extra}")
endforeach()
