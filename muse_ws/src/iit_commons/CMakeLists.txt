cmake_minimum_required(VERSION 3.10)
project(iit_commons)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)

set(BUILD_DOC OFF CACHE BOOL "Build Doxygen documentation")
if(BUILD_DOC)
  find_package(Doxygen QUIET)
  if(DOXYGEN_FOUND)
    configure_file(doc/Doxyfile.in ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile @ONLY)
    add_custom_target(doc_doxygen ALL
      COMMAND ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile
      WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
      COMMENT "Generating API documentation"
      VERBATIM)
  endif()
endif()

set(LIB_NAME iitcommons)
set(LIB_INSTALL_ROOT lib)
add_library(${LIB_NAME} SHARED
  src/hydraulics.cpp
  src/fbl/four_bar_linkage.cpp
  src/control/VirtualModel.cpp
  src/planning.cpp)
target_include_directories(${LIB_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(${LIB_NAME} PUBLIC Eigen3::Eigen)

include(yaml_config.cmake)
include(euler_angles.cmake)

install(DIRECTORY include/ DESTINATION include PATTERN ".svn" EXCLUDE)
install(TARGETS ${LIB_NAME}
  EXPORT export_${LIB_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

ament_export_targets(export_${LIB_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  foreach(test_name IN ITEMS vm algebra filt legboolmap planning rotation)
    if(test_name STREQUAL "vm")
      set(test_source src/control/testVirtualModel.cpp)
    elseif(test_name STREQUAL "algebra")
      set(test_source src/algebra_test.cpp)
    elseif(test_name STREQUAL "filt")
      set(test_source src/test-filters.cpp)
    elseif(test_name STREQUAL "legboolmap")
      set(test_source src/legboolmap_test.cpp)
    elseif(test_name STREQUAL "planning")
      set(test_source src/planning_test.cpp)
    else()
      set(test_source src/rotation_test.cpp)
    endif()
    ament_add_gtest(${test_name}-test ${test_source})
    if(TARGET ${test_name}-test)
      target_link_libraries(${test_name}-test ${LIB_NAME} Eigen3::Eigen pthread)
    endif()
  endforeach()
endif()

ament_package()
