cmake_minimum_required(VERSION 3.10)
project(state_estimator)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  rosconsole
  dynamic_reconfigure
  sensor_msgs
  geometry_msgs
  nav_msgs
  std_msgs
  tf2_ros
  pluginlib
  message_filters
  state_estimator_msgs
  iit_commons
)

find_package(Boost REQUIRED COMPONENTS
  chrono
  program_options
  system
  thread
)
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(PkgConfig REQUIRED)
find_package(pinocchio REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}_plugins
  CATKIN_DEPENDS
    roscpp
    roslib
    sensor_msgs
    geometry_msgs
    nav_msgs
    std_msgs
    tf2_ros
    pluginlib
    message_filters
    anymal_msgs
    state_estimator_msgs
    iit_commons
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIRS}
  ${PINOCCHIO_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}_plugins
  src/plugins/attitude_estimation_plugin.cpp
  src/plugins/contact_detection_plugin.cpp
  src/plugins/leg_odometry_plugin.cpp
  src/plugins/sensor_fusion_plugin.cpp
  src/plugins/tf_publisher_plugin.cpp
)
add_dependencies(${PROJECT_NAME}_plugins
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(${PROJECT_NAME}_plugins
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${PINOCCHIO_LIBRARIES}
  Eigen3::Eigen
)

add_executable(${PROJECT_NAME}_node
  src/state_estimator.cpp
  src/state_estimator_node.cpp
)
set_target_properties(${PROJECT_NAME}_node PROPERTIES
  OUTPUT_NAME node
  PREFIX ""
)
add_dependencies(${PROJECT_NAME}_node
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}_node ${PROJECT_NAME}_plugins
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
)

install(DIRECTORY config launch rviz urdfs
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES
  ${PROJECT_NAME}_plugins.xml
  ${PROJECT_NAME}_tools.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
