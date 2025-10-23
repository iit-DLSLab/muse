cmake_minimum_required(VERSION 3.5)
project(state_estimator)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(state_estimator_msgs REQUIRED)
find_package(iit_commons REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rosbag2_transport REQUIRED)
find_package(message_filters REQUIRED)

include_directories(include
  ${EIGEN3_INCLUDE_DIRS})


add_executable(state_estimator_node src/state_estimator_node.cpp)
ament_target_dependencies(state_estimator_node rclcpp std_msgs pluginlib)

# PLugins

#install headers
install(
  DIRECTORY include/
  DESTINATION include
)

install(FILES
  state_estimator_plugins.xml
  state_estimator_tools.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(
  include
)

#define plugins
add_library(state_estimator_plugins SHARED src/plugins/attitude_estimation_plugin.cpp
 src/plugins/contact_detection_plugin.cpp
 src/plugins/leg_odometry_plugin.cpp
 src/plugins/contact_data_packaging_plugin.cpp
 src/plugins/sensor_fusion_plugin.cpp)
target_include_directories(state_estimator_plugins PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${message_filters_INCLUDE_DIRS}
  ${state_estimator_msgs_INCLUDE_DIRS}
  ${iit_commons_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${nav_msgs_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${rosbag2_transport_INCLUDE_DIRS}
  ${pluginlib_INCLUDE_DIRS}
)

target_link_libraries(state_estimator_plugins PUBLIC
  ${state_estimator_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  message_filters::message_filters
  pluginlib::pluginlib
  rclcpp::rclcpp
  ${iit_commons_LIBRARIES}
)

pluginlib_export_plugin_description_file(state_estimator state_estimator_plugins.xml)

#copy dependencies to install dir
install(
  TARGETS state_estimator_plugins
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  state_estimator_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

ament_export_libraries(
  state_estimator_plugins
)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()