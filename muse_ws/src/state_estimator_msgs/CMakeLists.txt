cmake_minimum_required(VERSION 3.5)
set(CMAKE_CXX_STANDARD 17)
project(state_estimator_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  # "msg/YourMessageFile.msg"
  # Add all your .msg files here
  "msg/Attitude.msg"
  "msg/ContactDetection.msg"
  "msg/JointStateWithAcceleration.msg"
  "msg/LegOdometry.msg"
  "msg/SensorFusion.msg"
  DEPENDENCIES geometry_msgs sensor_msgs std_msgs
)
ament_export_dependencies(rosidl_default_runtime)

ament_package()