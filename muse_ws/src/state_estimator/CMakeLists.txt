cmake_minimum_required(VERSION 3.11)
project(state_estimator)

get_property(IS_MULTI_CONFIG GLOBAL PROPERTY GENERATOR_IS_MULTI_CONFIG)
if(NOT IS_MULTI_CONFIG AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS Debug Release RelWithDebInfo MinSizeRel)
  message(STATUS "${PROJECT_NAME}: CMAKE_BUILD_TYPE not set; defaulting to Release")
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(class_loader REQUIRED)
find_package(message_filters REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(state_estimator_msgs REQUIRED)
find_package(dls2_interface REQUIRED)
find_package(unitree_go REQUIRED)
find_package(unitree_hg REQUIRED)
find_package(iit_commons REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
execute_process(
  COMMAND "${Python3_EXECUTABLE}" -I -c "import numpy; print(numpy.get_include())"
  RESULT_VARIABLE NUMPY_QUERY_RESULT
  OUTPUT_VARIABLE DETECTED_NUMPY_INCLUDE_DIR
  OUTPUT_STRIP_TRAILING_WHITESPACE
)
if(NOT NUMPY_QUERY_RESULT EQUAL 0 OR NOT DETECTED_NUMPY_INCLUDE_DIR)
  message(FATAL_ERROR "Could not query NumPy include path from ${Python3_EXECUTABLE}")
endif()

# Pinocchio/EigenPy still consumes the legacy all-caps variables, while newer
# rosidl code consumes the FindPython variables. Keep both on the same Conda
# interpreter so dependency discovery cannot mix environments.
set(PYTHON_EXECUTABLE ${Python3_EXECUTABLE})
set(PYTHON_INCLUDE_DIRS ${Python3_INCLUDE_DIRS})
set(NUMPY_INCLUDE_DIRS ${DETECTED_NUMPY_INCLUDE_DIR})
set(Python_EXECUTABLE ${Python3_EXECUTABLE})
set(Python_INCLUDE_DIRS ${Python3_INCLUDE_DIRS})
set(Python_LIBRARIES ${Python3_LIBRARIES})
set(Python_NumPy_INCLUDE_DIR ${DETECTED_NUMPY_INCLUDE_DIR})
set(Python_NumPy_INCLUDE_DIRS ${DETECTED_NUMPY_INCLUDE_DIR})

# Some sourced ROS underlays prepend Python 3.12 packages even when CMake has
# selected the Conda Python 3.11 executable. Isolate EigenPy's import probe so
# it sees packages belonging to the selected interpreter.
set(SAVED_PYTHONPATH "$ENV{PYTHONPATH}")
set(ENV{PYTHONPATH} "")
find_package(pinocchio REQUIRED)
set(ENV{PYTHONPATH} "${SAVED_PYTHONPATH}")
find_package(yaml-cpp REQUIRED)

include_directories(include)
if(pinocchio_FOUND)
  include_directories(${PINOCCHIO_INCLUDE_DIRS})
endif()

# ── Plugin shared library ──────────────────────────────────────────────────
set(PLUGIN_SOURCES
  src/plugins/attitude_estimation_plugin.cpp
  src/plugins/contact_detection_plugin.cpp
  src/plugins/proprioceptive_sensor_fusion_plugin.cpp
)
if(pinocchio_FOUND)
  list(APPEND PLUGIN_SOURCES src/plugins/leg_odometry_plugin.cpp)
  message(STATUS "state_estimator: pinocchio found — building LegOdometryPlugin")
else()
  message(STATUS "state_estimator: pinocchio NOT found — skipping LegOdometryPlugin")
endif()

add_library(state_estimator_plugins SHARED ${PLUGIN_SOURCES})
target_include_directories(state_estimator_plugins PUBLIC 
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR}
)
ament_target_dependencies(state_estimator_plugins
  rclcpp
  pluginlib
  message_filters
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2_ros
  state_estimator_msgs
  dls2_interface
  unitree_go
  unitree_hg
  iit_commons
  ament_index_cpp
)
target_link_libraries(state_estimator_plugins Eigen3::Eigen yaml-cpp)
if(pinocchio_FOUND)
  target_link_libraries(state_estimator_plugins ${PINOCCHIO_LIBRARIES})
endif()

pluginlib_export_plugin_description_file(state_estimator state_estimator_plugins.xml)

# ── Main executable ────────────────────────────────────────────────────────
add_executable(state_estimator_node
  src/state_estimator.cpp
  src/state_estimator_node.cpp
)
target_include_directories(state_estimator_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR}
)
ament_target_dependencies(state_estimator_node
  rclcpp
  pluginlib
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2_ros
  state_estimator_msgs
  dls2_interface
  unitree_go
  unitree_hg
  iit_commons
  ament_index_cpp
)
target_link_libraries(state_estimator_node state_estimator_plugins Eigen3::Eigen yaml-cpp)
if(pinocchio_FOUND)
  target_link_libraries(state_estimator_node ${PINOCCHIO_LIBRARIES})
endif()

install(TARGETS state_estimator_node state_estimator_plugins
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY config launch urdfs
  DESTINATION share/${PROJECT_NAME}
)

install(FILES
  state_estimator_plugins.xml
  state_estimator_tools.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
