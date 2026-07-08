# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "series_elastic_actuator_msgs: 9 messages, 0 services")

set(MSG_I_FLAGS "-Iseries_elastic_actuator_msgs:/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg;-Isensor_msgs:/home/ynistico-iit.local/miniforge3/envs/muse/share/sensor_msgs/cmake/../msg;-Istd_msgs:/home/ynistico-iit.local/miniforge3/envs/muse/share/std_msgs/cmake/../msg;-Igeometry_msgs:/home/ynistico-iit.local/miniforge3/envs/muse/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(series_elastic_actuator_msgs_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorCommand.msg" NAME_WE)
add_custom_target(_series_elastic_actuator_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "series_elastic_actuator_msgs" "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorCommand.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorCommands.msg" NAME_WE)
add_custom_target(_series_elastic_actuator_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "series_elastic_actuator_msgs" "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorCommands.msg" "std_msgs/Header:series_elastic_actuator_msgs/SeActuatorCommand"
)

get_filename_component(_filename "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorReading.msg" NAME_WE)
add_custom_target(_series_elastic_actuator_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "series_elastic_actuator_msgs" "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorReading.msg" "series_elastic_actuator_msgs/SeActuatorCommand:series_elastic_actuator_msgs/SeActuatorState:std_msgs/Header:geometry_msgs/Quaternion:geometry_msgs/Vector3:sensor_msgs/Imu"
)

get_filename_component(_filename "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorReadingExtended.msg" NAME_WE)
add_custom_target(_series_elastic_actuator_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "series_elastic_actuator_msgs" "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorReadingExtended.msg" "series_elastic_actuator_msgs/SeActuatorCommand:std_msgs/Header:geometry_msgs/Quaternion:geometry_msgs/Vector3:series_elastic_actuator_msgs/SeActuatorStateExtended:sensor_msgs/Imu"
)

get_filename_component(_filename "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorReadings.msg" NAME_WE)
add_custom_target(_series_elastic_actuator_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "series_elastic_actuator_msgs" "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorReadings.msg" "series_elastic_actuator_msgs/SeActuatorCommand:series_elastic_actuator_msgs/SeActuatorState:std_msgs/Header:geometry_msgs/Quaternion:geometry_msgs/Vector3:series_elastic_actuator_msgs/SeActuatorReading:sensor_msgs/Imu"
)

get_filename_component(_filename "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorReadingsExtended.msg" NAME_WE)
add_custom_target(_series_elastic_actuator_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "series_elastic_actuator_msgs" "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorReadingsExtended.msg" "series_elastic_actuator_msgs/SeActuatorCommand:std_msgs/Header:series_elastic_actuator_msgs/SeActuatorReadingExtended:geometry_msgs/Quaternion:geometry_msgs/Vector3:series_elastic_actuator_msgs/SeActuatorStateExtended:sensor_msgs/Imu"
)

get_filename_component(_filename "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorState.msg" NAME_WE)
add_custom_target(_series_elastic_actuator_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "series_elastic_actuator_msgs" "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorState.msg" "std_msgs/Header:geometry_msgs/Quaternion:geometry_msgs/Vector3:sensor_msgs/Imu"
)

get_filename_component(_filename "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorStateExtended.msg" NAME_WE)
add_custom_target(_series_elastic_actuator_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "series_elastic_actuator_msgs" "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorStateExtended.msg" "std_msgs/Header:geometry_msgs/Quaternion:geometry_msgs/Vector3:sensor_msgs/Imu"
)

get_filename_component(_filename "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorStates.msg" NAME_WE)
add_custom_target(_series_elastic_actuator_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "series_elastic_actuator_msgs" "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorStates.msg" "series_elastic_actuator_msgs/SeActuatorState:std_msgs/Header:geometry_msgs/Quaternion:geometry_msgs/Vector3:sensor_msgs/Imu"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(series_elastic_actuator_msgs
  "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorCommand.msg"
  "${MSG_I_FLAGS}"
  "/home/ynistico-iit.local/miniforge3/envs/muse/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/series_elastic_actuator_msgs
)
_generate_msg_cpp(series_elastic_actuator_msgs
  "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorCommands.msg"
  "${MSG_I_FLAGS}"
  "/home/ynistico-iit.local/miniforge3/envs/muse/share/std_msgs/cmake/../msg/Header.msg;/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorCommand.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/series_elastic_actuator_msgs
)
_generate_msg_cpp(series_elastic_actuator_msgs
  "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorReading.msg"
  "${MSG_I_FLAGS}"
  "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorCommand.msg;/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorState.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/std_msgs/cmake/../msg/Header.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/geometry_msgs/cmake/../msg/Quaternion.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/geometry_msgs/cmake/../msg/Vector3.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/sensor_msgs/cmake/../msg/Imu.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/series_elastic_actuator_msgs
)
_generate_msg_cpp(series_elastic_actuator_msgs
  "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorReadingExtended.msg"
  "${MSG_I_FLAGS}"
  "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorCommand.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/std_msgs/cmake/../msg/Header.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/geometry_msgs/cmake/../msg/Quaternion.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/geometry_msgs/cmake/../msg/Vector3.msg;/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorStateExtended.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/sensor_msgs/cmake/../msg/Imu.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/series_elastic_actuator_msgs
)
_generate_msg_cpp(series_elastic_actuator_msgs
  "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorReadings.msg"
  "${MSG_I_FLAGS}"
  "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorCommand.msg;/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorState.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/std_msgs/cmake/../msg/Header.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/geometry_msgs/cmake/../msg/Quaternion.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/geometry_msgs/cmake/../msg/Vector3.msg;/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorReading.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/sensor_msgs/cmake/../msg/Imu.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/series_elastic_actuator_msgs
)
_generate_msg_cpp(series_elastic_actuator_msgs
  "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorReadingsExtended.msg"
  "${MSG_I_FLAGS}"
  "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorCommand.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/std_msgs/cmake/../msg/Header.msg;/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorReadingExtended.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/geometry_msgs/cmake/../msg/Quaternion.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/geometry_msgs/cmake/../msg/Vector3.msg;/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorStateExtended.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/sensor_msgs/cmake/../msg/Imu.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/series_elastic_actuator_msgs
)
_generate_msg_cpp(series_elastic_actuator_msgs
  "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorState.msg"
  "${MSG_I_FLAGS}"
  "/home/ynistico-iit.local/miniforge3/envs/muse/share/std_msgs/cmake/../msg/Header.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/geometry_msgs/cmake/../msg/Quaternion.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/geometry_msgs/cmake/../msg/Vector3.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/sensor_msgs/cmake/../msg/Imu.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/series_elastic_actuator_msgs
)
_generate_msg_cpp(series_elastic_actuator_msgs
  "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorStateExtended.msg"
  "${MSG_I_FLAGS}"
  "/home/ynistico-iit.local/miniforge3/envs/muse/share/std_msgs/cmake/../msg/Header.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/geometry_msgs/cmake/../msg/Quaternion.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/geometry_msgs/cmake/../msg/Vector3.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/sensor_msgs/cmake/../msg/Imu.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/series_elastic_actuator_msgs
)
_generate_msg_cpp(series_elastic_actuator_msgs
  "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorStates.msg"
  "${MSG_I_FLAGS}"
  "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorState.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/std_msgs/cmake/../msg/Header.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/geometry_msgs/cmake/../msg/Quaternion.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/geometry_msgs/cmake/../msg/Vector3.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/sensor_msgs/cmake/../msg/Imu.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/series_elastic_actuator_msgs
)

### Generating Services

### Generating Module File
_generate_module_cpp(series_elastic_actuator_msgs
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/series_elastic_actuator_msgs
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(series_elastic_actuator_msgs_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(series_elastic_actuator_msgs_generate_messages series_elastic_actuator_msgs_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorCommand.msg" NAME_WE)
add_dependencies(series_elastic_actuator_msgs_generate_messages_cpp _series_elastic_actuator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorCommands.msg" NAME_WE)
add_dependencies(series_elastic_actuator_msgs_generate_messages_cpp _series_elastic_actuator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorReading.msg" NAME_WE)
add_dependencies(series_elastic_actuator_msgs_generate_messages_cpp _series_elastic_actuator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorReadingExtended.msg" NAME_WE)
add_dependencies(series_elastic_actuator_msgs_generate_messages_cpp _series_elastic_actuator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorReadings.msg" NAME_WE)
add_dependencies(series_elastic_actuator_msgs_generate_messages_cpp _series_elastic_actuator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorReadingsExtended.msg" NAME_WE)
add_dependencies(series_elastic_actuator_msgs_generate_messages_cpp _series_elastic_actuator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorState.msg" NAME_WE)
add_dependencies(series_elastic_actuator_msgs_generate_messages_cpp _series_elastic_actuator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorStateExtended.msg" NAME_WE)
add_dependencies(series_elastic_actuator_msgs_generate_messages_cpp _series_elastic_actuator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorStates.msg" NAME_WE)
add_dependencies(series_elastic_actuator_msgs_generate_messages_cpp _series_elastic_actuator_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(series_elastic_actuator_msgs_gencpp)
add_dependencies(series_elastic_actuator_msgs_gencpp series_elastic_actuator_msgs_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS series_elastic_actuator_msgs_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(series_elastic_actuator_msgs
  "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorCommand.msg"
  "${MSG_I_FLAGS}"
  "/home/ynistico-iit.local/miniforge3/envs/muse/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/series_elastic_actuator_msgs
)
_generate_msg_eus(series_elastic_actuator_msgs
  "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorCommands.msg"
  "${MSG_I_FLAGS}"
  "/home/ynistico-iit.local/miniforge3/envs/muse/share/std_msgs/cmake/../msg/Header.msg;/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorCommand.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/series_elastic_actuator_msgs
)
_generate_msg_eus(series_elastic_actuator_msgs
  "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorReading.msg"
  "${MSG_I_FLAGS}"
  "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorCommand.msg;/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorState.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/std_msgs/cmake/../msg/Header.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/geometry_msgs/cmake/../msg/Quaternion.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/geometry_msgs/cmake/../msg/Vector3.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/sensor_msgs/cmake/../msg/Imu.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/series_elastic_actuator_msgs
)
_generate_msg_eus(series_elastic_actuator_msgs
  "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorReadingExtended.msg"
  "${MSG_I_FLAGS}"
  "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorCommand.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/std_msgs/cmake/../msg/Header.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/geometry_msgs/cmake/../msg/Quaternion.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/geometry_msgs/cmake/../msg/Vector3.msg;/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorStateExtended.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/sensor_msgs/cmake/../msg/Imu.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/series_elastic_actuator_msgs
)
_generate_msg_eus(series_elastic_actuator_msgs
  "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorReadings.msg"
  "${MSG_I_FLAGS}"
  "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorCommand.msg;/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorState.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/std_msgs/cmake/../msg/Header.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/geometry_msgs/cmake/../msg/Quaternion.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/geometry_msgs/cmake/../msg/Vector3.msg;/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorReading.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/sensor_msgs/cmake/../msg/Imu.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/series_elastic_actuator_msgs
)
_generate_msg_eus(series_elastic_actuator_msgs
  "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorReadingsExtended.msg"
  "${MSG_I_FLAGS}"
  "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorCommand.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/std_msgs/cmake/../msg/Header.msg;/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorReadingExtended.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/geometry_msgs/cmake/../msg/Quaternion.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/geometry_msgs/cmake/../msg/Vector3.msg;/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorStateExtended.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/sensor_msgs/cmake/../msg/Imu.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/series_elastic_actuator_msgs
)
_generate_msg_eus(series_elastic_actuator_msgs
  "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorState.msg"
  "${MSG_I_FLAGS}"
  "/home/ynistico-iit.local/miniforge3/envs/muse/share/std_msgs/cmake/../msg/Header.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/geometry_msgs/cmake/../msg/Quaternion.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/geometry_msgs/cmake/../msg/Vector3.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/sensor_msgs/cmake/../msg/Imu.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/series_elastic_actuator_msgs
)
_generate_msg_eus(series_elastic_actuator_msgs
  "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorStateExtended.msg"
  "${MSG_I_FLAGS}"
  "/home/ynistico-iit.local/miniforge3/envs/muse/share/std_msgs/cmake/../msg/Header.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/geometry_msgs/cmake/../msg/Quaternion.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/geometry_msgs/cmake/../msg/Vector3.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/sensor_msgs/cmake/../msg/Imu.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/series_elastic_actuator_msgs
)
_generate_msg_eus(series_elastic_actuator_msgs
  "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorStates.msg"
  "${MSG_I_FLAGS}"
  "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorState.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/std_msgs/cmake/../msg/Header.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/geometry_msgs/cmake/../msg/Quaternion.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/geometry_msgs/cmake/../msg/Vector3.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/sensor_msgs/cmake/../msg/Imu.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/series_elastic_actuator_msgs
)

### Generating Services

### Generating Module File
_generate_module_eus(series_elastic_actuator_msgs
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/series_elastic_actuator_msgs
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(series_elastic_actuator_msgs_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(series_elastic_actuator_msgs_generate_messages series_elastic_actuator_msgs_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorCommand.msg" NAME_WE)
add_dependencies(series_elastic_actuator_msgs_generate_messages_eus _series_elastic_actuator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorCommands.msg" NAME_WE)
add_dependencies(series_elastic_actuator_msgs_generate_messages_eus _series_elastic_actuator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorReading.msg" NAME_WE)
add_dependencies(series_elastic_actuator_msgs_generate_messages_eus _series_elastic_actuator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorReadingExtended.msg" NAME_WE)
add_dependencies(series_elastic_actuator_msgs_generate_messages_eus _series_elastic_actuator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorReadings.msg" NAME_WE)
add_dependencies(series_elastic_actuator_msgs_generate_messages_eus _series_elastic_actuator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorReadingsExtended.msg" NAME_WE)
add_dependencies(series_elastic_actuator_msgs_generate_messages_eus _series_elastic_actuator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorState.msg" NAME_WE)
add_dependencies(series_elastic_actuator_msgs_generate_messages_eus _series_elastic_actuator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorStateExtended.msg" NAME_WE)
add_dependencies(series_elastic_actuator_msgs_generate_messages_eus _series_elastic_actuator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorStates.msg" NAME_WE)
add_dependencies(series_elastic_actuator_msgs_generate_messages_eus _series_elastic_actuator_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(series_elastic_actuator_msgs_geneus)
add_dependencies(series_elastic_actuator_msgs_geneus series_elastic_actuator_msgs_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS series_elastic_actuator_msgs_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(series_elastic_actuator_msgs
  "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorCommand.msg"
  "${MSG_I_FLAGS}"
  "/home/ynistico-iit.local/miniforge3/envs/muse/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/series_elastic_actuator_msgs
)
_generate_msg_lisp(series_elastic_actuator_msgs
  "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorCommands.msg"
  "${MSG_I_FLAGS}"
  "/home/ynistico-iit.local/miniforge3/envs/muse/share/std_msgs/cmake/../msg/Header.msg;/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorCommand.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/series_elastic_actuator_msgs
)
_generate_msg_lisp(series_elastic_actuator_msgs
  "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorReading.msg"
  "${MSG_I_FLAGS}"
  "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorCommand.msg;/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorState.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/std_msgs/cmake/../msg/Header.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/geometry_msgs/cmake/../msg/Quaternion.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/geometry_msgs/cmake/../msg/Vector3.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/sensor_msgs/cmake/../msg/Imu.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/series_elastic_actuator_msgs
)
_generate_msg_lisp(series_elastic_actuator_msgs
  "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorReadingExtended.msg"
  "${MSG_I_FLAGS}"
  "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorCommand.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/std_msgs/cmake/../msg/Header.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/geometry_msgs/cmake/../msg/Quaternion.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/geometry_msgs/cmake/../msg/Vector3.msg;/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorStateExtended.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/sensor_msgs/cmake/../msg/Imu.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/series_elastic_actuator_msgs
)
_generate_msg_lisp(series_elastic_actuator_msgs
  "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorReadings.msg"
  "${MSG_I_FLAGS}"
  "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorCommand.msg;/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorState.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/std_msgs/cmake/../msg/Header.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/geometry_msgs/cmake/../msg/Quaternion.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/geometry_msgs/cmake/../msg/Vector3.msg;/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorReading.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/sensor_msgs/cmake/../msg/Imu.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/series_elastic_actuator_msgs
)
_generate_msg_lisp(series_elastic_actuator_msgs
  "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorReadingsExtended.msg"
  "${MSG_I_FLAGS}"
  "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorCommand.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/std_msgs/cmake/../msg/Header.msg;/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorReadingExtended.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/geometry_msgs/cmake/../msg/Quaternion.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/geometry_msgs/cmake/../msg/Vector3.msg;/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorStateExtended.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/sensor_msgs/cmake/../msg/Imu.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/series_elastic_actuator_msgs
)
_generate_msg_lisp(series_elastic_actuator_msgs
  "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorState.msg"
  "${MSG_I_FLAGS}"
  "/home/ynistico-iit.local/miniforge3/envs/muse/share/std_msgs/cmake/../msg/Header.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/geometry_msgs/cmake/../msg/Quaternion.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/geometry_msgs/cmake/../msg/Vector3.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/sensor_msgs/cmake/../msg/Imu.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/series_elastic_actuator_msgs
)
_generate_msg_lisp(series_elastic_actuator_msgs
  "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorStateExtended.msg"
  "${MSG_I_FLAGS}"
  "/home/ynistico-iit.local/miniforge3/envs/muse/share/std_msgs/cmake/../msg/Header.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/geometry_msgs/cmake/../msg/Quaternion.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/geometry_msgs/cmake/../msg/Vector3.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/sensor_msgs/cmake/../msg/Imu.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/series_elastic_actuator_msgs
)
_generate_msg_lisp(series_elastic_actuator_msgs
  "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorStates.msg"
  "${MSG_I_FLAGS}"
  "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorState.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/std_msgs/cmake/../msg/Header.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/geometry_msgs/cmake/../msg/Quaternion.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/geometry_msgs/cmake/../msg/Vector3.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/sensor_msgs/cmake/../msg/Imu.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/series_elastic_actuator_msgs
)

### Generating Services

### Generating Module File
_generate_module_lisp(series_elastic_actuator_msgs
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/series_elastic_actuator_msgs
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(series_elastic_actuator_msgs_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(series_elastic_actuator_msgs_generate_messages series_elastic_actuator_msgs_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorCommand.msg" NAME_WE)
add_dependencies(series_elastic_actuator_msgs_generate_messages_lisp _series_elastic_actuator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorCommands.msg" NAME_WE)
add_dependencies(series_elastic_actuator_msgs_generate_messages_lisp _series_elastic_actuator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorReading.msg" NAME_WE)
add_dependencies(series_elastic_actuator_msgs_generate_messages_lisp _series_elastic_actuator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorReadingExtended.msg" NAME_WE)
add_dependencies(series_elastic_actuator_msgs_generate_messages_lisp _series_elastic_actuator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorReadings.msg" NAME_WE)
add_dependencies(series_elastic_actuator_msgs_generate_messages_lisp _series_elastic_actuator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorReadingsExtended.msg" NAME_WE)
add_dependencies(series_elastic_actuator_msgs_generate_messages_lisp _series_elastic_actuator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorState.msg" NAME_WE)
add_dependencies(series_elastic_actuator_msgs_generate_messages_lisp _series_elastic_actuator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorStateExtended.msg" NAME_WE)
add_dependencies(series_elastic_actuator_msgs_generate_messages_lisp _series_elastic_actuator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorStates.msg" NAME_WE)
add_dependencies(series_elastic_actuator_msgs_generate_messages_lisp _series_elastic_actuator_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(series_elastic_actuator_msgs_genlisp)
add_dependencies(series_elastic_actuator_msgs_genlisp series_elastic_actuator_msgs_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS series_elastic_actuator_msgs_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(series_elastic_actuator_msgs
  "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorCommand.msg"
  "${MSG_I_FLAGS}"
  "/home/ynistico-iit.local/miniforge3/envs/muse/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/series_elastic_actuator_msgs
)
_generate_msg_nodejs(series_elastic_actuator_msgs
  "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorCommands.msg"
  "${MSG_I_FLAGS}"
  "/home/ynistico-iit.local/miniforge3/envs/muse/share/std_msgs/cmake/../msg/Header.msg;/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorCommand.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/series_elastic_actuator_msgs
)
_generate_msg_nodejs(series_elastic_actuator_msgs
  "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorReading.msg"
  "${MSG_I_FLAGS}"
  "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorCommand.msg;/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorState.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/std_msgs/cmake/../msg/Header.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/geometry_msgs/cmake/../msg/Quaternion.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/geometry_msgs/cmake/../msg/Vector3.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/sensor_msgs/cmake/../msg/Imu.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/series_elastic_actuator_msgs
)
_generate_msg_nodejs(series_elastic_actuator_msgs
  "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorReadingExtended.msg"
  "${MSG_I_FLAGS}"
  "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorCommand.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/std_msgs/cmake/../msg/Header.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/geometry_msgs/cmake/../msg/Quaternion.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/geometry_msgs/cmake/../msg/Vector3.msg;/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorStateExtended.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/sensor_msgs/cmake/../msg/Imu.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/series_elastic_actuator_msgs
)
_generate_msg_nodejs(series_elastic_actuator_msgs
  "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorReadings.msg"
  "${MSG_I_FLAGS}"
  "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorCommand.msg;/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorState.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/std_msgs/cmake/../msg/Header.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/geometry_msgs/cmake/../msg/Quaternion.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/geometry_msgs/cmake/../msg/Vector3.msg;/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorReading.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/sensor_msgs/cmake/../msg/Imu.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/series_elastic_actuator_msgs
)
_generate_msg_nodejs(series_elastic_actuator_msgs
  "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorReadingsExtended.msg"
  "${MSG_I_FLAGS}"
  "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorCommand.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/std_msgs/cmake/../msg/Header.msg;/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorReadingExtended.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/geometry_msgs/cmake/../msg/Quaternion.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/geometry_msgs/cmake/../msg/Vector3.msg;/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorStateExtended.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/sensor_msgs/cmake/../msg/Imu.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/series_elastic_actuator_msgs
)
_generate_msg_nodejs(series_elastic_actuator_msgs
  "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorState.msg"
  "${MSG_I_FLAGS}"
  "/home/ynistico-iit.local/miniforge3/envs/muse/share/std_msgs/cmake/../msg/Header.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/geometry_msgs/cmake/../msg/Quaternion.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/geometry_msgs/cmake/../msg/Vector3.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/sensor_msgs/cmake/../msg/Imu.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/series_elastic_actuator_msgs
)
_generate_msg_nodejs(series_elastic_actuator_msgs
  "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorStateExtended.msg"
  "${MSG_I_FLAGS}"
  "/home/ynistico-iit.local/miniforge3/envs/muse/share/std_msgs/cmake/../msg/Header.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/geometry_msgs/cmake/../msg/Quaternion.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/geometry_msgs/cmake/../msg/Vector3.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/sensor_msgs/cmake/../msg/Imu.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/series_elastic_actuator_msgs
)
_generate_msg_nodejs(series_elastic_actuator_msgs
  "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorStates.msg"
  "${MSG_I_FLAGS}"
  "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorState.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/std_msgs/cmake/../msg/Header.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/geometry_msgs/cmake/../msg/Quaternion.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/geometry_msgs/cmake/../msg/Vector3.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/sensor_msgs/cmake/../msg/Imu.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/series_elastic_actuator_msgs
)

### Generating Services

### Generating Module File
_generate_module_nodejs(series_elastic_actuator_msgs
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/series_elastic_actuator_msgs
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(series_elastic_actuator_msgs_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(series_elastic_actuator_msgs_generate_messages series_elastic_actuator_msgs_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorCommand.msg" NAME_WE)
add_dependencies(series_elastic_actuator_msgs_generate_messages_nodejs _series_elastic_actuator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorCommands.msg" NAME_WE)
add_dependencies(series_elastic_actuator_msgs_generate_messages_nodejs _series_elastic_actuator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorReading.msg" NAME_WE)
add_dependencies(series_elastic_actuator_msgs_generate_messages_nodejs _series_elastic_actuator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorReadingExtended.msg" NAME_WE)
add_dependencies(series_elastic_actuator_msgs_generate_messages_nodejs _series_elastic_actuator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorReadings.msg" NAME_WE)
add_dependencies(series_elastic_actuator_msgs_generate_messages_nodejs _series_elastic_actuator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorReadingsExtended.msg" NAME_WE)
add_dependencies(series_elastic_actuator_msgs_generate_messages_nodejs _series_elastic_actuator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorState.msg" NAME_WE)
add_dependencies(series_elastic_actuator_msgs_generate_messages_nodejs _series_elastic_actuator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorStateExtended.msg" NAME_WE)
add_dependencies(series_elastic_actuator_msgs_generate_messages_nodejs _series_elastic_actuator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorStates.msg" NAME_WE)
add_dependencies(series_elastic_actuator_msgs_generate_messages_nodejs _series_elastic_actuator_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(series_elastic_actuator_msgs_gennodejs)
add_dependencies(series_elastic_actuator_msgs_gennodejs series_elastic_actuator_msgs_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS series_elastic_actuator_msgs_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(series_elastic_actuator_msgs
  "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorCommand.msg"
  "${MSG_I_FLAGS}"
  "/home/ynistico-iit.local/miniforge3/envs/muse/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/series_elastic_actuator_msgs
)
_generate_msg_py(series_elastic_actuator_msgs
  "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorCommands.msg"
  "${MSG_I_FLAGS}"
  "/home/ynistico-iit.local/miniforge3/envs/muse/share/std_msgs/cmake/../msg/Header.msg;/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorCommand.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/series_elastic_actuator_msgs
)
_generate_msg_py(series_elastic_actuator_msgs
  "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorReading.msg"
  "${MSG_I_FLAGS}"
  "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorCommand.msg;/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorState.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/std_msgs/cmake/../msg/Header.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/geometry_msgs/cmake/../msg/Quaternion.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/geometry_msgs/cmake/../msg/Vector3.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/sensor_msgs/cmake/../msg/Imu.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/series_elastic_actuator_msgs
)
_generate_msg_py(series_elastic_actuator_msgs
  "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorReadingExtended.msg"
  "${MSG_I_FLAGS}"
  "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorCommand.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/std_msgs/cmake/../msg/Header.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/geometry_msgs/cmake/../msg/Quaternion.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/geometry_msgs/cmake/../msg/Vector3.msg;/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorStateExtended.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/sensor_msgs/cmake/../msg/Imu.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/series_elastic_actuator_msgs
)
_generate_msg_py(series_elastic_actuator_msgs
  "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorReadings.msg"
  "${MSG_I_FLAGS}"
  "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorCommand.msg;/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorState.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/std_msgs/cmake/../msg/Header.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/geometry_msgs/cmake/../msg/Quaternion.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/geometry_msgs/cmake/../msg/Vector3.msg;/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorReading.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/sensor_msgs/cmake/../msg/Imu.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/series_elastic_actuator_msgs
)
_generate_msg_py(series_elastic_actuator_msgs
  "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorReadingsExtended.msg"
  "${MSG_I_FLAGS}"
  "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorCommand.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/std_msgs/cmake/../msg/Header.msg;/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorReadingExtended.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/geometry_msgs/cmake/../msg/Quaternion.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/geometry_msgs/cmake/../msg/Vector3.msg;/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorStateExtended.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/sensor_msgs/cmake/../msg/Imu.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/series_elastic_actuator_msgs
)
_generate_msg_py(series_elastic_actuator_msgs
  "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorState.msg"
  "${MSG_I_FLAGS}"
  "/home/ynistico-iit.local/miniforge3/envs/muse/share/std_msgs/cmake/../msg/Header.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/geometry_msgs/cmake/../msg/Quaternion.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/geometry_msgs/cmake/../msg/Vector3.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/sensor_msgs/cmake/../msg/Imu.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/series_elastic_actuator_msgs
)
_generate_msg_py(series_elastic_actuator_msgs
  "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorStateExtended.msg"
  "${MSG_I_FLAGS}"
  "/home/ynistico-iit.local/miniforge3/envs/muse/share/std_msgs/cmake/../msg/Header.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/geometry_msgs/cmake/../msg/Quaternion.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/geometry_msgs/cmake/../msg/Vector3.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/sensor_msgs/cmake/../msg/Imu.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/series_elastic_actuator_msgs
)
_generate_msg_py(series_elastic_actuator_msgs
  "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorStates.msg"
  "${MSG_I_FLAGS}"
  "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorState.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/std_msgs/cmake/../msg/Header.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/geometry_msgs/cmake/../msg/Quaternion.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/geometry_msgs/cmake/../msg/Vector3.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/sensor_msgs/cmake/../msg/Imu.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/series_elastic_actuator_msgs
)

### Generating Services

### Generating Module File
_generate_module_py(series_elastic_actuator_msgs
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/series_elastic_actuator_msgs
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(series_elastic_actuator_msgs_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(series_elastic_actuator_msgs_generate_messages series_elastic_actuator_msgs_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorCommand.msg" NAME_WE)
add_dependencies(series_elastic_actuator_msgs_generate_messages_py _series_elastic_actuator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorCommands.msg" NAME_WE)
add_dependencies(series_elastic_actuator_msgs_generate_messages_py _series_elastic_actuator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorReading.msg" NAME_WE)
add_dependencies(series_elastic_actuator_msgs_generate_messages_py _series_elastic_actuator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorReadingExtended.msg" NAME_WE)
add_dependencies(series_elastic_actuator_msgs_generate_messages_py _series_elastic_actuator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorReadings.msg" NAME_WE)
add_dependencies(series_elastic_actuator_msgs_generate_messages_py _series_elastic_actuator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorReadingsExtended.msg" NAME_WE)
add_dependencies(series_elastic_actuator_msgs_generate_messages_py _series_elastic_actuator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorState.msg" NAME_WE)
add_dependencies(series_elastic_actuator_msgs_generate_messages_py _series_elastic_actuator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorStateExtended.msg" NAME_WE)
add_dependencies(series_elastic_actuator_msgs_generate_messages_py _series_elastic_actuator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorStates.msg" NAME_WE)
add_dependencies(series_elastic_actuator_msgs_generate_messages_py _series_elastic_actuator_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(series_elastic_actuator_msgs_genpy)
add_dependencies(series_elastic_actuator_msgs_genpy series_elastic_actuator_msgs_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS series_elastic_actuator_msgs_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/series_elastic_actuator_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/series_elastic_actuator_msgs
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET sensor_msgs_generate_messages_cpp)
  add_dependencies(series_elastic_actuator_msgs_generate_messages_cpp sensor_msgs_generate_messages_cpp)
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(series_elastic_actuator_msgs_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/series_elastic_actuator_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/series_elastic_actuator_msgs
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET sensor_msgs_generate_messages_eus)
  add_dependencies(series_elastic_actuator_msgs_generate_messages_eus sensor_msgs_generate_messages_eus)
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(series_elastic_actuator_msgs_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/series_elastic_actuator_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/series_elastic_actuator_msgs
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET sensor_msgs_generate_messages_lisp)
  add_dependencies(series_elastic_actuator_msgs_generate_messages_lisp sensor_msgs_generate_messages_lisp)
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(series_elastic_actuator_msgs_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/series_elastic_actuator_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/series_elastic_actuator_msgs
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET sensor_msgs_generate_messages_nodejs)
  add_dependencies(series_elastic_actuator_msgs_generate_messages_nodejs sensor_msgs_generate_messages_nodejs)
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(series_elastic_actuator_msgs_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/series_elastic_actuator_msgs)
  install(CODE "execute_process(COMMAND \"/home/ynistico-iit.local/miniforge3/envs/muse/bin/python3.11\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/series_elastic_actuator_msgs\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/series_elastic_actuator_msgs
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET sensor_msgs_generate_messages_py)
  add_dependencies(series_elastic_actuator_msgs_generate_messages_py sensor_msgs_generate_messages_py)
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(series_elastic_actuator_msgs_generate_messages_py std_msgs_generate_messages_py)
endif()
