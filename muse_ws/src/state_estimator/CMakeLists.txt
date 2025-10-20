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
find_package(Eigen3 REQUIRED)
find_package(rosbag2_transport REQUIRED)

include_directories(include
  ${EIGEN3_INCLUDE_DIRS})

add_executable(talker src/state_estimator_node.cpp)
ament_target_dependencies(talker rclcpp std_msgs pluginlib)

# PLugins

#install headers
install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(
  include
)

#define plugins
add_library(attitude_estimation_plugin src/plugins/attitude_estimation_plugin.cpp)
target_include_directories(attitude_estimation_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${state_estimator_msgs_INCLUDE_DIRS}
  ${iit_commons_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${nav_msgs_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${rosbag2_transport_INCLUDE_DIRS}
  ${pluginlib_INCLUDE_DIRS}
)

target_link_libraries(attitude_estimation_plugin PUBLIC
  ${state_estimator_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${pluginlib_LIBRARIES}
  pluginlib::pluginlib
  iit_commons
  ${iit_commons_LIBRARIES}
  rclcpp::rclcpp
)

#copy dependencies to install dir
install(
  TARGETS attitude_estimation_plugin
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  talker
  DESTINATION lib/${PROJECT_NAME})

ament_package()