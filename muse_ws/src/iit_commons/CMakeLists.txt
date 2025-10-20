# Project configuration
cmake_minimum_required(VERSION 3.5)

project(iit_commons)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()



find_package(PkgConfig)
pkg_check_modules(EIGEN3 eigen3 REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

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

set(LIB_NAME iit_commons)

set(SOURCES src/hydraulics.cpp 
            src/fbl/four_bar_linkage.cpp
            src/control/VirtualModel.cpp
			src/planning.cpp)

# Include directories
include_directories(include ${EIGEN3_INCLUDE_DIRS})

# Add library
add_library(${LIB_NAME} SHARED ${SOURCES})

# Sub-libraries
#
# The YAML C++ configuration facilities.
include(yaml_config.cmake)

# The Euler-angles transforms
include(euler_angles.cmake)

install(
    DIRECTORY include/
    DESTINATION include
    PATTERN ".svn" EXCLUDE
)

install(TARGETS iit_commons
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)


if (CATKIN_ENABLE_TESTING)
	find_package(catkin REQUIRED COMPONENTS rostest)
	include_directories(include ${catkin_INCLUDE_DIRS})

	catkin_add_gtest(vm-test src/control/testVirtualModel.cpp)
	target_link_libraries(vm-test ${LIB_NAME} ${catkin_LIBRARIES} pthread)

	catkin_add_gtest(algebra-test src/algebra_test.cpp)
	target_link_libraries(algebra-test ${catkin_LIBRARIES} pthread)

	catkin_add_gtest(filt-test src/test-filters.cpp)
	target_link_libraries(filt-test ${LIB_NAME} ${catkin_LIBRARIES})

	catkin_add_gtest(legboolmap-test src/legboolmap_test.cpp)
	target_link_libraries(legboolmap-test ${LIB_NAME} ${catkin_LIBRARIES})

	catkin_add_gtest(planning-test src/planning_test.cpp)
	target_link_libraries(planning-test ${LIB_NAME} ${catkin_LIBRARIES} pthread)

	catkin_add_gtest(rotation-test src/rotation_test.cpp)
	target_link_libraries(rotation-test ${LIB_NAME} ${catkin_LIBRARIES} pthread)

endif()

ament_export_include_directories(include)
ament_export_libraries(iit_commons)
ament_export_dependencies(rclcpp std_msgs)

ament_package()
