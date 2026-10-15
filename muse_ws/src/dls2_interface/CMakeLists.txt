cmake_minimum_required(VERSION 3.11)
project(dls2_interface)

get_property(IS_MULTI_CONFIG GLOBAL PROPERTY GENERATOR_IS_MULTI_CONFIG)
if(NOT IS_MULTI_CONFIG AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS Debug Release RelWithDebInfo MinSizeRel)
  message(STATUS "${PROJECT_NAME}: CMAKE_BUILD_TYPE not set; defaulting to Release")
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development NumPy)

set(Python_EXECUTABLE ${Python3_EXECUTABLE} CACHE FILEPATH "" FORCE)
set(Python_INCLUDE_DIRS ${Python3_INCLUDE_DIRS} CACHE STRING "" FORCE)
set(Python_LIBRARIES ${Python3_LIBRARIES} CACHE STRING "" FORCE)
set(Python_NumPy_INCLUDE_DIRS ${Python3_NumPy_INCLUDE_DIRS} CACHE STRING "" FORCE)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Pose.msg"
  "msg/Screw.msg"
  "msg/BaseState.msg"
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
