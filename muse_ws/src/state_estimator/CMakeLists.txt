cmake_minimum_required(VERSION 3.5)
project(state_estimator)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED
)
find_package(rclcpp)
find_package(sensor_msgs)
find_package(geometry_msgs)
find_package(nav_msgs)
find_package(pluginlib)
find_package(state_estimator_msgs)
find_package(std_msgs)
find_package(iit_commons)
find_package(rosidl_default_generators REQUIRED)


find_package(Boost REQUIRED COMPONENTS system program_options)

find_package(Eigen3 REQUIRED NO_MODULE)

find_package(PkgConfig REQUIRED)

find_package(pinocchio REQUIRED)



## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  /usr/include/eigen3  
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

include_directories(${PINOCCHIO_INCLUDE_DIRS})


## Declare a C++ library
add_library(${PROJECT_NAME}_plugins
    src/plugins/attitude_estimation_plugin.cpp
    src/plugins/contact_detection_plugin.cpp
    src/plugins/leg_odometry_plugin.cpp
    src/plugins/sensor_fusion_plugin.cpp
    # src/Models/sensor_fusion.cpp 
)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide

add_executable(${PROJECT_NAME}_node src/state_estimator.cpp src/state_estimator_node.cpp)

## Rename C++ executable without prefix
set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

target_link_libraries(state_estimator_node
  rclcpp::rclcpp
  ${std_msgs_TARGETS}
)

#TODO
#target_link_libraries(${PROJECT_NAME}_plugins
#  ${catkin_LIBRARIES}  # <-- this includes message_filters now
#  ${PINOCCHIO_LIBRARIES}
#)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES
  ${PROJECT_NAME}_plugins.xml #${PROJECT_NAME}_tools.xml
  DESTINATION lib/${PROJECT_NAME}
)
 
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.hpp"
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE)


if (CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)

    include_directories( include
      ${catkin_INCLUDE_DIRS}
      ${Eigen_INCLUDE_DIRS}
    )

endif()

ament_package()