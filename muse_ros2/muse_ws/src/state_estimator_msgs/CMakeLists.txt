cmake_minimum_required(VERSION 3.8)
project(state_estimator_msgs)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Attitude.msg"
  "msg/ContactDetection.msg"
  "msg/LegOdometry.msg"
  "msg/SensorFusion.msg"
  "msg/JointStateWithAcceleration.msg"
  "srv/GetActiveEstimators.srv"
  "srv/GetBlacklist.srv"
  "srv/GetEstimatorDescription.srv"
  "srv/GetWhitelist.srv"
  "srv/ListAllEstimators.srv"
  "srv/PauseEstimator.srv"
  "srv/ResetEstimator.srv"
  "srv/RestartEstimator.srv"
  "srv/ResumeEstimator.srv"
  "srv/StartEstimator.srv"
  "srv/StopEstimator.srv"
  DEPENDENCIES geometry_msgs sensor_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
