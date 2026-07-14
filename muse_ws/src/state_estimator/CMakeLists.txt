cmake_minimum_required(VERSION 3.10)
project(state_estimator)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(iit_commons REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(state_estimator_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)

add_library(${PROJECT_NAME}_plugins SHARED
  src/plugins/attitude_estimation_plugin.cpp
  src/plugins/contact_detection_plugin.cpp
  src/plugins/leg_odometry_plugin.cpp
  src/plugins/sensor_fusion_plugin.cpp
  src/plugins/tf_publisher_plugin.cpp)
target_include_directories(${PROJECT_NAME}_plugins PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${PROJECT_NAME}_plugins
  ament_index_cpp geometry_msgs iit_commons nav_msgs pluginlib rclcpp
  sensor_msgs state_estimator_msgs tf2_ros)
target_link_libraries(${PROJECT_NAME}_plugins Eigen3::Eigen)
if(TARGET pinocchio::pinocchio)
  target_link_libraries(${PROJECT_NAME}_plugins pinocchio::pinocchio)
else()
  target_include_directories(${PROJECT_NAME}_plugins PRIVATE ${PINOCCHIO_INCLUDE_DIRS})
  target_link_libraries(${PROJECT_NAME}_plugins ${PINOCCHIO_LIBRARIES})
endif()

add_executable(${PROJECT_NAME}_node
  src/state_estimator.cpp
  src/state_estimator_node.cpp)
target_include_directories(${PROJECT_NAME}_node PRIVATE include)
ament_target_dependencies(${PROJECT_NAME}_node pluginlib rclcpp state_estimator_msgs)

pluginlib_export_plugin_description_file(${PROJECT_NAME} state_estimator_plugins.xml)

install(TARGETS ${PROJECT_NAME}_node ${PROJECT_NAME}_plugins
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch rviz urdfs DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(model_validation_test test/model_validation_test.cpp)
  if(TARGET model_validation_test)
    target_include_directories(model_validation_test PRIVATE include)
    target_compile_definitions(model_validation_test PRIVATE
      TEST_URDF_PATH="${CMAKE_CURRENT_SOURCE_DIR}/urdfs/anymal.urdf")
    target_link_libraries(model_validation_test Eigen3::Eigen)
    if(TARGET pinocchio::pinocchio)
      target_link_libraries(model_validation_test pinocchio::pinocchio)
    else()
      target_include_directories(model_validation_test PRIVATE ${PINOCCHIO_INCLUDE_DIRS})
      target_link_libraries(model_validation_test ${PINOCCHIO_LIBRARIES})
    endif()
  endif()

  ament_add_gtest(plugin_loading_test test/plugin_loading_test.cpp)
  if(TARGET plugin_loading_test)
    target_include_directories(plugin_loading_test PRIVATE include)
    ament_target_dependencies(plugin_loading_test pluginlib rclcpp)
  endif()
endif()

ament_export_include_directories(include)
ament_export_dependencies(rclcpp)
ament_package()
