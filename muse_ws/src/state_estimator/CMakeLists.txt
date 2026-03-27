cmake_minimum_required(VERSION 3.10)
project(state_estimator)

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
find_package(iit_commons REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(pinocchio QUIET)
find_package(yaml-cpp REQUIRED)

include_directories(
  include
  /usr/include/eigen3
)
if(pinocchio_FOUND)
  include_directories(${PINOCCHIO_INCLUDE_DIRS})
endif()

# ── Plugin shared library ──────────────────────────────────────────────────
set(PLUGIN_SOURCES
  src/plugins/attitude_estimation_plugin.cpp
  src/plugins/contact_detection_plugin.cpp
  src/plugins/sensor_fusion_plugin.cpp
)
if(pinocchio_FOUND)
  list(APPEND PLUGIN_SOURCES src/plugins/leg_odometry_plugin.cpp)
  message(STATUS "state_estimator: pinocchio found — building LegOdometryPlugin")
else()
  message(STATUS "state_estimator: pinocchio NOT found — skipping LegOdometryPlugin")
endif()

add_library(state_estimator_plugins SHARED ${PLUGIN_SOURCES})
ament_target_dependencies(state_estimator_plugins
  rclcpp
  pluginlib
  message_filters
  sensor_msgs
  geometry_msgs
  nav_msgs
  state_estimator_msgs
  dls2_interface
  unitree_go
  iit_commons
  ament_index_cpp
)
target_link_libraries(state_estimator_plugins yaml-cpp)
if(pinocchio_FOUND)
  target_link_libraries(state_estimator_plugins ${PINOCCHIO_LIBRARIES})
endif()

pluginlib_export_plugin_description_file(state_estimator state_estimator_plugins.xml)

# ── Main executable ────────────────────────────────────────────────────────
add_executable(state_estimator_node
  src/state_estimator.cpp
  src/state_estimator_node.cpp
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
  iit_commons
  ament_index_cpp
)
target_link_libraries(state_estimator_node state_estimator_plugins yaml-cpp)
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
