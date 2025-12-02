# Project configuration
cmake_minimum_required(VERSION 3.8)
project(iit_commons)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

set(BUILD_DOC false  CACHE BOOL "Build Doxygen documentation")

if(${BUILD_DOC})
    # check if Doxygen is installed
    find_package(Doxygen QUIET)
    if (DOXYGEN_FOUND)
        # set input and output files
        set(DOXYGEN_IN ${CMAKE_CURRENT_SOURCE_DIR}/doc/Doxyfile.in)
        set(DOXYGEN_OUT ${CMAKE_CURRENT_SOURCE_DIR}/doc/Doxyfile)

        # request to configure the file
        configure_file(${DOXYGEN_IN} ${DOXYGEN_OUT} @ONLY)

        # note the option ALL which allows to build the docs together with the application
        add_custom_target( doc_doxygen ALL
            COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_OUT}
            WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
            COMMENT "Generating API documentation with Doxygen"
            VERBATIM )
    else (DOXYGEN_FOUND)
        message(WARNING "Doxygen need to be installed to generate the Doxygen documentation")
    endif (DOXYGEN_FOUND)
endif()

set(LIB_NAME iitcommons)

set(SOURCES src/hydraulics.cpp 
            src/fbl/four_bar_linkage.cpp
            src/control/VirtualModel.cpp
			src/planning.cpp)

# Include directories
include_directories(include)

# Add library
add_library(${LIB_NAME} SHARED ${SOURCES})

# Specify dependencies for the library
ament_target_dependencies(${LIB_NAME}
  Eigen3
)

# Sub-libraries
#
# The YAML C++ configuration facilities.
include(yaml_config.cmake)

# The Euler-angles transforms
include(euler_angles.cmake)

# Install library
install(TARGETS ${LIB_NAME}
  EXPORT ${LIB_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install headers
install(DIRECTORY include/
  DESTINATION include
  PATTERN ".svn" EXCLUDE
)

# Export the library
ament_export_targets(${LIB_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(${LIB_NAME})
ament_export_dependencies(Eigen3)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  
  ament_add_gtest(vm-test src/control/testVirtualModel.cpp)
  target_link_libraries(vm-test ${LIB_NAME} pthread)
  ament_target_dependencies(vm-test Eigen3)

  ament_add_gtest(algebra-test src/algebra_test.cpp)
  target_link_libraries(algebra-test pthread)
  ament_target_dependencies(algebra-test Eigen3)

  ament_add_gtest(filt-test src/test-filters.cpp)
  target_link_libraries(filt-test ${LIB_NAME})
  ament_target_dependencies(filt-test Eigen3)

  ament_add_gtest(legboolmap-test src/legboolmap_test.cpp)
  target_link_libraries(legboolmap-test ${LIB_NAME})
  ament_target_dependencies(legboolmap-test Eigen3)

  ament_add_gtest(planning-test src/planning_test.cpp)
  target_link_libraries(planning-test ${LIB_NAME} pthread)
  ament_target_dependencies(planning-test Eigen3)

  ament_add_gtest(rotation-test src/rotation_test.cpp)
  target_link_libraries(rotation-test ${LIB_NAME} pthread)
  ament_target_dependencies(rotation-test Eigen3)
endif()

ament_package()

