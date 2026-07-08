cmake_minimum_required(VERSION 3.10)
project(state_estimator_msgs)

find_package(catkin REQUIRED COMPONENTS geometry_msgs sensor_msgs std_msgs message_generation)

include_directories(include)

add_message_files(
  DIRECTORY msg
  FILES
  	attitude.msg
	ContactDetection.msg
	LegOdometry.msg
	sensor_fusion.msg
	JointStateWithAcceleration.msg
)

## Generate services in the 'srv' folder
add_service_files(
  DIRECTORY srv
  FILES
	getActiveEstimators.srv
	getBlacklist.srv
	getEstimatorDescription.srv
	getWhitelist.srv
	listAllEstimators.srv
	pauseEstimator.srv
	resetEstimator.srv
	restartEstimator.srv
	resumeEstimator.srv
	startEstimator.srv
	stopEstimator.srv
 )

generate_messages(DEPENDENCIES geometry_msgs sensor_msgs std_msgs)

catkin_package(CATKIN_DEPENDS geometry_msgs sensor_msgs std_msgs message_runtime)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h")
