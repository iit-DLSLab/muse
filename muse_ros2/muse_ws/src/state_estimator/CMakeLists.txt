cmake_minimum_required(VERSION 3.8)
project(state_estimator)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(message_filters REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(state_estimator_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)
# find_package(iit_commons REQUIRED)

find_package(Boost REQUIRED COMPONENTS system program_options)
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(PkgConfig REQUIRED)
find_package(pinocchio REQUIRED)


################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
# rosidl_generate_interfaces(${PROJECT_NAME}
#   "msg/RollPitch.msg"
#   "msg/RollPitchStamped.msg"
#   DEPENDENCIES std_msgs sensor_msgs nav_msgs state_estimator_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

# ROS 2 uses parameters instead of dynamic_reconfigure
# Parameters are declared and handled in the node code


###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include
  /usr/include/eigen3  
)

include_directories(${PINOCCHIO_INCLUDE_DIRS})


## Declare a C++ library
add_library(${PROJECT_NAME}_plugins SHARED
    src/plugins/attitude_estimation_plugin.cpp
    src/plugins/contact_detection_plugin.cpp
    src/plugins/leg_odometry_plugin.cpp
    src/plugins/sensor_fusion_plugin.cpp
    # src/Models/sensor_fusion.cpp 
)

# Specify dependencies for the library
ament_target_dependencies(${PROJECT_NAME}_plugins
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  pluginlib
  message_filters
  state_estimator_msgs
  std_msgs
  # iit_commons
)

target_link_libraries(${PROJECT_NAME}_plugins
  ${PINOCCHIO_LIBRARIES}
  Eigen3::Eigen
)

## Declare a C++ executable
add_executable(${PROJECT_NAME}_node src/state_estimator.cpp src/state_estimator_node.cpp)

# Specify dependencies for the executable
ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  pluginlib
  message_filters
  state_estimator_msgs
  std_msgs
  # iit_commons
)

target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}_plugins
)

## Installation
install(TARGETS
  ${PROJECT_NAME}_plugins
  ${PROJECT_NAME}_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(FILES
  ${PROJECT_NAME}_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)
 
install(
  DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  FILES_MATCHING
  PATTERN "*.launch"
  PATTERN "*.launch.xml"
  PATTERN "*.yaml"
)

# Install Python launch files with executable permissions
install(PROGRAMS 
  launch/state_estimator_xml.launch.py
  launch/state_estimator.launch.py
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY urdfs/
  DESTINATION share/${PROJECT_NAME}/urdfs
)

# Export plugin description
pluginlib_export_plugin_description_file(${PROJECT_NAME} ${PROJECT_NAME}_plugins.xml)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Export dependencies
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_plugins)
ament_export_dependencies(
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  pluginlib
  message_filters
  state_estimator_msgs
  std_msgs
  # iit_commons
)

ament_package()
