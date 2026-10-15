cmake_minimum_required(VERSION 3.11)
project(state_estimator_msgs)

get_property(IS_MULTI_CONFIG GLOBAL PROPERTY GENERATOR_IS_MULTI_CONFIG)
if(NOT IS_MULTI_CONFIG AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS Debug Release RelWithDebInfo MinSizeRel)
  message(STATUS "${PROJECT_NAME}: CMAKE_BUILD_TYPE not set; defaulting to Release")
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development NumPy)

set(Python_EXECUTABLE ${Python3_EXECUTABLE} CACHE FILEPATH "" FORCE)
set(Python_INCLUDE_DIRS ${Python3_INCLUDE_DIRS} CACHE STRING "" FORCE)
set(Python_LIBRARIES ${Python3_LIBRARIES} CACHE STRING "" FORCE)
set(Python_NumPy_INCLUDE_DIRS ${Python3_NumPy_INCLUDE_DIRS} CACHE STRING "" FORCE)

include_directories(include)

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

install(DIRECTORY include/${PROJECT_NAME}/
	DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.h")

ament_export_dependencies(rosidl_default_runtime)
ament_package()
