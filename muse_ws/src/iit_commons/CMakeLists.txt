cmake_minimum_required(VERSION 3.8)
project(iit_commons)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gtest QUIET)

find_package(PkgConfig)
pkg_check_modules(EIGEN3 eigen3 REQUIRED)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

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
set(LIB_INSTALL_ROOT lib)

set(SOURCES src/hydraulics.cpp 
            src/fbl/four_bar_linkage.cpp
            src/control/VirtualModel.cpp
			src/planning.cpp)

# Include directories
include_directories(include ${EIGEN3_INCLUDE_DIRS})

# Add library
add_library(${LIB_NAME} SHARED ${SOURCES})
target_include_directories(${LIB_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${EIGEN3_INCLUDE_DIRS}
)

# Sub-libraries
#
# The YAML C++ configuration facilities.
include(yaml_config.cmake)

# The Euler-angles transforms
include(euler_angles.cmake)

install(DIRECTORY include/
  DESTINATION include
  PATTERN ".svn" EXCLUDE
)

install(TARGETS ${LIB_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

ament_export_include_directories(include)
ament_export_libraries(${LIB_NAME})

if (BUILD_TESTING AND ament_cmake_gtest_FOUND)
    ament_add_gtest(vm-test src/control/testVirtualModel.cpp)
    if(TARGET vm-test)
        target_link_libraries(vm-test ${LIB_NAME} pthread)
    endif()

    ament_add_gtest(algebra-test src/algebra_test.cpp)
    if(TARGET algebra-test)
        target_link_libraries(algebra-test pthread)
    endif()

    ament_add_gtest(filt-test src/test-filters.cpp)
    if(TARGET filt-test)
        target_link_libraries(filt-test ${LIB_NAME})
    endif()

    ament_add_gtest(legboolmap-test src/legboolmap_test.cpp)
    if(TARGET legboolmap-test)
        target_link_libraries(legboolmap-test ${LIB_NAME})
    endif()

    ament_add_gtest(planning-test src/planning_test.cpp)
    if(TARGET planning-test)
        target_link_libraries(planning-test ${LIB_NAME} pthread)
    endif()

    ament_add_gtest(rotation-test src/rotation_test.cpp)
    if(TARGET rotation-test)
        target_link_libraries(rotation-test ${LIB_NAME} pthread)
    endif()
endif()

ament_package()

