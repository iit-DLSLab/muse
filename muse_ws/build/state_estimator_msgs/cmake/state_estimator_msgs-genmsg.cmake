# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "state_estimator_msgs: 5 messages, 11 services")

set(MSG_I_FLAGS "-Istate_estimator_msgs:/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/msg;-Igeometry_msgs:/home/ynistico-iit.local/miniforge3/envs/muse/share/geometry_msgs/cmake/../msg;-Isensor_msgs:/home/ynistico-iit.local/miniforge3/envs/muse/share/sensor_msgs/cmake/../msg;-Istd_msgs:/home/ynistico-iit.local/miniforge3/envs/muse/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(state_estimator_msgs_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/msg/attitude.msg" NAME_WE)
add_custom_target(_state_estimator_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "state_estimator_msgs" "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/msg/attitude.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/msg/ContactDetection.msg" NAME_WE)
add_custom_target(_state_estimator_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "state_estimator_msgs" "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/msg/ContactDetection.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/msg/LegOdometry.msg" NAME_WE)
add_custom_target(_state_estimator_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "state_estimator_msgs" "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/msg/LegOdometry.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/msg/sensor_fusion.msg" NAME_WE)
add_custom_target(_state_estimator_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "state_estimator_msgs" "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/msg/sensor_fusion.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/msg/JointStateWithAcceleration.msg" NAME_WE)
add_custom_target(_state_estimator_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "state_estimator_msgs" "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/msg/JointStateWithAcceleration.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/srv/getActiveEstimators.srv" NAME_WE)
add_custom_target(_state_estimator_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "state_estimator_msgs" "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/srv/getActiveEstimators.srv" ""
)

get_filename_component(_filename "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/srv/getBlacklist.srv" NAME_WE)
add_custom_target(_state_estimator_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "state_estimator_msgs" "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/srv/getBlacklist.srv" ""
)

get_filename_component(_filename "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/srv/getEstimatorDescription.srv" NAME_WE)
add_custom_target(_state_estimator_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "state_estimator_msgs" "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/srv/getEstimatorDescription.srv" ""
)

get_filename_component(_filename "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/srv/getWhitelist.srv" NAME_WE)
add_custom_target(_state_estimator_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "state_estimator_msgs" "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/srv/getWhitelist.srv" ""
)

get_filename_component(_filename "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/srv/listAllEstimators.srv" NAME_WE)
add_custom_target(_state_estimator_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "state_estimator_msgs" "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/srv/listAllEstimators.srv" ""
)

get_filename_component(_filename "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/srv/pauseEstimator.srv" NAME_WE)
add_custom_target(_state_estimator_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "state_estimator_msgs" "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/srv/pauseEstimator.srv" ""
)

get_filename_component(_filename "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/srv/resetEstimator.srv" NAME_WE)
add_custom_target(_state_estimator_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "state_estimator_msgs" "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/srv/resetEstimator.srv" ""
)

get_filename_component(_filename "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/srv/restartEstimator.srv" NAME_WE)
add_custom_target(_state_estimator_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "state_estimator_msgs" "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/srv/restartEstimator.srv" ""
)

get_filename_component(_filename "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/srv/resumeEstimator.srv" NAME_WE)
add_custom_target(_state_estimator_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "state_estimator_msgs" "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/srv/resumeEstimator.srv" ""
)

get_filename_component(_filename "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/srv/startEstimator.srv" NAME_WE)
add_custom_target(_state_estimator_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "state_estimator_msgs" "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/srv/startEstimator.srv" ""
)

get_filename_component(_filename "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/srv/stopEstimator.srv" NAME_WE)
add_custom_target(_state_estimator_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "state_estimator_msgs" "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/srv/stopEstimator.srv" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(state_estimator_msgs
  "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/msg/attitude.msg"
  "${MSG_I_FLAGS}"
  "/home/ynistico-iit.local/miniforge3/envs/muse/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/state_estimator_msgs
)
_generate_msg_cpp(state_estimator_msgs
  "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/msg/ContactDetection.msg"
  "${MSG_I_FLAGS}"
  "/home/ynistico-iit.local/miniforge3/envs/muse/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/state_estimator_msgs
)
_generate_msg_cpp(state_estimator_msgs
  "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/msg/LegOdometry.msg"
  "${MSG_I_FLAGS}"
  "/home/ynistico-iit.local/miniforge3/envs/muse/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/state_estimator_msgs
)
_generate_msg_cpp(state_estimator_msgs
  "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/msg/sensor_fusion.msg"
  "${MSG_I_FLAGS}"
  "/home/ynistico-iit.local/miniforge3/envs/muse/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/state_estimator_msgs
)
_generate_msg_cpp(state_estimator_msgs
  "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/msg/JointStateWithAcceleration.msg"
  "${MSG_I_FLAGS}"
  "/home/ynistico-iit.local/miniforge3/envs/muse/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/state_estimator_msgs
)

### Generating Services
_generate_srv_cpp(state_estimator_msgs
  "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/srv/getActiveEstimators.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/state_estimator_msgs
)
_generate_srv_cpp(state_estimator_msgs
  "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/srv/getBlacklist.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/state_estimator_msgs
)
_generate_srv_cpp(state_estimator_msgs
  "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/srv/getEstimatorDescription.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/state_estimator_msgs
)
_generate_srv_cpp(state_estimator_msgs
  "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/srv/getWhitelist.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/state_estimator_msgs
)
_generate_srv_cpp(state_estimator_msgs
  "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/srv/listAllEstimators.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/state_estimator_msgs
)
_generate_srv_cpp(state_estimator_msgs
  "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/srv/pauseEstimator.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/state_estimator_msgs
)
_generate_srv_cpp(state_estimator_msgs
  "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/srv/resetEstimator.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/state_estimator_msgs
)
_generate_srv_cpp(state_estimator_msgs
  "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/srv/restartEstimator.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/state_estimator_msgs
)
_generate_srv_cpp(state_estimator_msgs
  "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/srv/resumeEstimator.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/state_estimator_msgs
)
_generate_srv_cpp(state_estimator_msgs
  "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/srv/startEstimator.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/state_estimator_msgs
)
_generate_srv_cpp(state_estimator_msgs
  "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/srv/stopEstimator.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/state_estimator_msgs
)

### Generating Module File
_generate_module_cpp(state_estimator_msgs
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/state_estimator_msgs
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(state_estimator_msgs_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(state_estimator_msgs_generate_messages state_estimator_msgs_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/msg/attitude.msg" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_cpp _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/msg/ContactDetection.msg" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_cpp _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/msg/LegOdometry.msg" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_cpp _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/msg/sensor_fusion.msg" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_cpp _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/msg/JointStateWithAcceleration.msg" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_cpp _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/srv/getActiveEstimators.srv" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_cpp _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/srv/getBlacklist.srv" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_cpp _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/srv/getEstimatorDescription.srv" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_cpp _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/srv/getWhitelist.srv" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_cpp _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/srv/listAllEstimators.srv" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_cpp _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/srv/pauseEstimator.srv" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_cpp _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/srv/resetEstimator.srv" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_cpp _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/srv/restartEstimator.srv" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_cpp _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/srv/resumeEstimator.srv" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_cpp _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/srv/startEstimator.srv" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_cpp _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/srv/stopEstimator.srv" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_cpp _state_estimator_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(state_estimator_msgs_gencpp)
add_dependencies(state_estimator_msgs_gencpp state_estimator_msgs_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS state_estimator_msgs_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(state_estimator_msgs
  "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/msg/attitude.msg"
  "${MSG_I_FLAGS}"
  "/home/ynistico-iit.local/miniforge3/envs/muse/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/state_estimator_msgs
)
_generate_msg_eus(state_estimator_msgs
  "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/msg/ContactDetection.msg"
  "${MSG_I_FLAGS}"
  "/home/ynistico-iit.local/miniforge3/envs/muse/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/state_estimator_msgs
)
_generate_msg_eus(state_estimator_msgs
  "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/msg/LegOdometry.msg"
  "${MSG_I_FLAGS}"
  "/home/ynistico-iit.local/miniforge3/envs/muse/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/state_estimator_msgs
)
_generate_msg_eus(state_estimator_msgs
  "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/msg/sensor_fusion.msg"
  "${MSG_I_FLAGS}"
  "/home/ynistico-iit.local/miniforge3/envs/muse/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/state_estimator_msgs
)
_generate_msg_eus(state_estimator_msgs
  "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/msg/JointStateWithAcceleration.msg"
  "${MSG_I_FLAGS}"
  "/home/ynistico-iit.local/miniforge3/envs/muse/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/state_estimator_msgs
)

### Generating Services
_generate_srv_eus(state_estimator_msgs
  "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/srv/getActiveEstimators.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/state_estimator_msgs
)
_generate_srv_eus(state_estimator_msgs
  "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/srv/getBlacklist.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/state_estimator_msgs
)
_generate_srv_eus(state_estimator_msgs
  "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/srv/getEstimatorDescription.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/state_estimator_msgs
)
_generate_srv_eus(state_estimator_msgs
  "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/srv/getWhitelist.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/state_estimator_msgs
)
_generate_srv_eus(state_estimator_msgs
  "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/srv/listAllEstimators.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/state_estimator_msgs
)
_generate_srv_eus(state_estimator_msgs
  "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/srv/pauseEstimator.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/state_estimator_msgs
)
_generate_srv_eus(state_estimator_msgs
  "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/srv/resetEstimator.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/state_estimator_msgs
)
_generate_srv_eus(state_estimator_msgs
  "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/srv/restartEstimator.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/state_estimator_msgs
)
_generate_srv_eus(state_estimator_msgs
  "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/srv/resumeEstimator.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/state_estimator_msgs
)
_generate_srv_eus(state_estimator_msgs
  "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/srv/startEstimator.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/state_estimator_msgs
)
_generate_srv_eus(state_estimator_msgs
  "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/srv/stopEstimator.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/state_estimator_msgs
)

### Generating Module File
_generate_module_eus(state_estimator_msgs
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/state_estimator_msgs
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(state_estimator_msgs_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(state_estimator_msgs_generate_messages state_estimator_msgs_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/msg/attitude.msg" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_eus _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/msg/ContactDetection.msg" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_eus _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/msg/LegOdometry.msg" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_eus _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/msg/sensor_fusion.msg" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_eus _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/msg/JointStateWithAcceleration.msg" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_eus _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/srv/getActiveEstimators.srv" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_eus _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/srv/getBlacklist.srv" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_eus _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/srv/getEstimatorDescription.srv" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_eus _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/srv/getWhitelist.srv" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_eus _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/srv/listAllEstimators.srv" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_eus _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/srv/pauseEstimator.srv" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_eus _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/srv/resetEstimator.srv" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_eus _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/srv/restartEstimator.srv" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_eus _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/srv/resumeEstimator.srv" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_eus _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/srv/startEstimator.srv" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_eus _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/srv/stopEstimator.srv" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_eus _state_estimator_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(state_estimator_msgs_geneus)
add_dependencies(state_estimator_msgs_geneus state_estimator_msgs_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS state_estimator_msgs_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(state_estimator_msgs
  "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/msg/attitude.msg"
  "${MSG_I_FLAGS}"
  "/home/ynistico-iit.local/miniforge3/envs/muse/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/state_estimator_msgs
)
_generate_msg_lisp(state_estimator_msgs
  "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/msg/ContactDetection.msg"
  "${MSG_I_FLAGS}"
  "/home/ynistico-iit.local/miniforge3/envs/muse/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/state_estimator_msgs
)
_generate_msg_lisp(state_estimator_msgs
  "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/msg/LegOdometry.msg"
  "${MSG_I_FLAGS}"
  "/home/ynistico-iit.local/miniforge3/envs/muse/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/state_estimator_msgs
)
_generate_msg_lisp(state_estimator_msgs
  "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/msg/sensor_fusion.msg"
  "${MSG_I_FLAGS}"
  "/home/ynistico-iit.local/miniforge3/envs/muse/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/state_estimator_msgs
)
_generate_msg_lisp(state_estimator_msgs
  "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/msg/JointStateWithAcceleration.msg"
  "${MSG_I_FLAGS}"
  "/home/ynistico-iit.local/miniforge3/envs/muse/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/state_estimator_msgs
)

### Generating Services
_generate_srv_lisp(state_estimator_msgs
  "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/srv/getActiveEstimators.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/state_estimator_msgs
)
_generate_srv_lisp(state_estimator_msgs
  "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/srv/getBlacklist.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/state_estimator_msgs
)
_generate_srv_lisp(state_estimator_msgs
  "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/srv/getEstimatorDescription.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/state_estimator_msgs
)
_generate_srv_lisp(state_estimator_msgs
  "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/srv/getWhitelist.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/state_estimator_msgs
)
_generate_srv_lisp(state_estimator_msgs
  "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/srv/listAllEstimators.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/state_estimator_msgs
)
_generate_srv_lisp(state_estimator_msgs
  "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/srv/pauseEstimator.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/state_estimator_msgs
)
_generate_srv_lisp(state_estimator_msgs
  "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/srv/resetEstimator.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/state_estimator_msgs
)
_generate_srv_lisp(state_estimator_msgs
  "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/srv/restartEstimator.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/state_estimator_msgs
)
_generate_srv_lisp(state_estimator_msgs
  "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/srv/resumeEstimator.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/state_estimator_msgs
)
_generate_srv_lisp(state_estimator_msgs
  "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/srv/startEstimator.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/state_estimator_msgs
)
_generate_srv_lisp(state_estimator_msgs
  "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/srv/stopEstimator.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/state_estimator_msgs
)

### Generating Module File
_generate_module_lisp(state_estimator_msgs
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/state_estimator_msgs
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(state_estimator_msgs_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(state_estimator_msgs_generate_messages state_estimator_msgs_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/msg/attitude.msg" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_lisp _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/msg/ContactDetection.msg" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_lisp _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/msg/LegOdometry.msg" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_lisp _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/msg/sensor_fusion.msg" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_lisp _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/msg/JointStateWithAcceleration.msg" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_lisp _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/srv/getActiveEstimators.srv" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_lisp _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/srv/getBlacklist.srv" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_lisp _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/srv/getEstimatorDescription.srv" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_lisp _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/srv/getWhitelist.srv" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_lisp _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/srv/listAllEstimators.srv" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_lisp _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/srv/pauseEstimator.srv" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_lisp _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/srv/resetEstimator.srv" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_lisp _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/srv/restartEstimator.srv" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_lisp _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/srv/resumeEstimator.srv" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_lisp _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/srv/startEstimator.srv" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_lisp _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/srv/stopEstimator.srv" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_lisp _state_estimator_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(state_estimator_msgs_genlisp)
add_dependencies(state_estimator_msgs_genlisp state_estimator_msgs_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS state_estimator_msgs_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(state_estimator_msgs
  "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/msg/attitude.msg"
  "${MSG_I_FLAGS}"
  "/home/ynistico-iit.local/miniforge3/envs/muse/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/state_estimator_msgs
)
_generate_msg_nodejs(state_estimator_msgs
  "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/msg/ContactDetection.msg"
  "${MSG_I_FLAGS}"
  "/home/ynistico-iit.local/miniforge3/envs/muse/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/state_estimator_msgs
)
_generate_msg_nodejs(state_estimator_msgs
  "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/msg/LegOdometry.msg"
  "${MSG_I_FLAGS}"
  "/home/ynistico-iit.local/miniforge3/envs/muse/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/state_estimator_msgs
)
_generate_msg_nodejs(state_estimator_msgs
  "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/msg/sensor_fusion.msg"
  "${MSG_I_FLAGS}"
  "/home/ynistico-iit.local/miniforge3/envs/muse/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/state_estimator_msgs
)
_generate_msg_nodejs(state_estimator_msgs
  "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/msg/JointStateWithAcceleration.msg"
  "${MSG_I_FLAGS}"
  "/home/ynistico-iit.local/miniforge3/envs/muse/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/state_estimator_msgs
)

### Generating Services
_generate_srv_nodejs(state_estimator_msgs
  "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/srv/getActiveEstimators.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/state_estimator_msgs
)
_generate_srv_nodejs(state_estimator_msgs
  "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/srv/getBlacklist.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/state_estimator_msgs
)
_generate_srv_nodejs(state_estimator_msgs
  "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/srv/getEstimatorDescription.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/state_estimator_msgs
)
_generate_srv_nodejs(state_estimator_msgs
  "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/srv/getWhitelist.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/state_estimator_msgs
)
_generate_srv_nodejs(state_estimator_msgs
  "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/srv/listAllEstimators.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/state_estimator_msgs
)
_generate_srv_nodejs(state_estimator_msgs
  "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/srv/pauseEstimator.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/state_estimator_msgs
)
_generate_srv_nodejs(state_estimator_msgs
  "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/srv/resetEstimator.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/state_estimator_msgs
)
_generate_srv_nodejs(state_estimator_msgs
  "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/srv/restartEstimator.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/state_estimator_msgs
)
_generate_srv_nodejs(state_estimator_msgs
  "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/srv/resumeEstimator.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/state_estimator_msgs
)
_generate_srv_nodejs(state_estimator_msgs
  "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/srv/startEstimator.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/state_estimator_msgs
)
_generate_srv_nodejs(state_estimator_msgs
  "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/srv/stopEstimator.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/state_estimator_msgs
)

### Generating Module File
_generate_module_nodejs(state_estimator_msgs
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/state_estimator_msgs
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(state_estimator_msgs_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(state_estimator_msgs_generate_messages state_estimator_msgs_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/msg/attitude.msg" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_nodejs _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/msg/ContactDetection.msg" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_nodejs _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/msg/LegOdometry.msg" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_nodejs _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/msg/sensor_fusion.msg" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_nodejs _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/msg/JointStateWithAcceleration.msg" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_nodejs _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/srv/getActiveEstimators.srv" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_nodejs _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/srv/getBlacklist.srv" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_nodejs _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/srv/getEstimatorDescription.srv" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_nodejs _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/srv/getWhitelist.srv" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_nodejs _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/srv/listAllEstimators.srv" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_nodejs _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/srv/pauseEstimator.srv" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_nodejs _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/srv/resetEstimator.srv" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_nodejs _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/srv/restartEstimator.srv" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_nodejs _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/srv/resumeEstimator.srv" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_nodejs _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/srv/startEstimator.srv" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_nodejs _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/srv/stopEstimator.srv" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_nodejs _state_estimator_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(state_estimator_msgs_gennodejs)
add_dependencies(state_estimator_msgs_gennodejs state_estimator_msgs_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS state_estimator_msgs_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(state_estimator_msgs
  "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/msg/attitude.msg"
  "${MSG_I_FLAGS}"
  "/home/ynistico-iit.local/miniforge3/envs/muse/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/state_estimator_msgs
)
_generate_msg_py(state_estimator_msgs
  "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/msg/ContactDetection.msg"
  "${MSG_I_FLAGS}"
  "/home/ynistico-iit.local/miniforge3/envs/muse/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/state_estimator_msgs
)
_generate_msg_py(state_estimator_msgs
  "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/msg/LegOdometry.msg"
  "${MSG_I_FLAGS}"
  "/home/ynistico-iit.local/miniforge3/envs/muse/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/state_estimator_msgs
)
_generate_msg_py(state_estimator_msgs
  "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/msg/sensor_fusion.msg"
  "${MSG_I_FLAGS}"
  "/home/ynistico-iit.local/miniforge3/envs/muse/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/state_estimator_msgs
)
_generate_msg_py(state_estimator_msgs
  "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/msg/JointStateWithAcceleration.msg"
  "${MSG_I_FLAGS}"
  "/home/ynistico-iit.local/miniforge3/envs/muse/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/state_estimator_msgs
)

### Generating Services
_generate_srv_py(state_estimator_msgs
  "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/srv/getActiveEstimators.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/state_estimator_msgs
)
_generate_srv_py(state_estimator_msgs
  "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/srv/getBlacklist.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/state_estimator_msgs
)
_generate_srv_py(state_estimator_msgs
  "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/srv/getEstimatorDescription.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/state_estimator_msgs
)
_generate_srv_py(state_estimator_msgs
  "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/srv/getWhitelist.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/state_estimator_msgs
)
_generate_srv_py(state_estimator_msgs
  "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/srv/listAllEstimators.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/state_estimator_msgs
)
_generate_srv_py(state_estimator_msgs
  "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/srv/pauseEstimator.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/state_estimator_msgs
)
_generate_srv_py(state_estimator_msgs
  "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/srv/resetEstimator.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/state_estimator_msgs
)
_generate_srv_py(state_estimator_msgs
  "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/srv/restartEstimator.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/state_estimator_msgs
)
_generate_srv_py(state_estimator_msgs
  "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/srv/resumeEstimator.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/state_estimator_msgs
)
_generate_srv_py(state_estimator_msgs
  "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/srv/startEstimator.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/state_estimator_msgs
)
_generate_srv_py(state_estimator_msgs
  "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/srv/stopEstimator.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/state_estimator_msgs
)

### Generating Module File
_generate_module_py(state_estimator_msgs
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/state_estimator_msgs
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(state_estimator_msgs_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(state_estimator_msgs_generate_messages state_estimator_msgs_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/msg/attitude.msg" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_py _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/msg/ContactDetection.msg" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_py _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/msg/LegOdometry.msg" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_py _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/msg/sensor_fusion.msg" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_py _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/msg/JointStateWithAcceleration.msg" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_py _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/srv/getActiveEstimators.srv" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_py _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/srv/getBlacklist.srv" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_py _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/srv/getEstimatorDescription.srv" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_py _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/srv/getWhitelist.srv" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_py _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/srv/listAllEstimators.srv" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_py _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/srv/pauseEstimator.srv" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_py _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/srv/resetEstimator.srv" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_py _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/srv/restartEstimator.srv" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_py _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/srv/resumeEstimator.srv" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_py _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/srv/startEstimator.srv" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_py _state_estimator_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/state_estimator_msgs/srv/stopEstimator.srv" NAME_WE)
add_dependencies(state_estimator_msgs_generate_messages_py _state_estimator_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(state_estimator_msgs_genpy)
add_dependencies(state_estimator_msgs_genpy state_estimator_msgs_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS state_estimator_msgs_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/state_estimator_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/state_estimator_msgs
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(state_estimator_msgs_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()
if(TARGET sensor_msgs_generate_messages_cpp)
  add_dependencies(state_estimator_msgs_generate_messages_cpp sensor_msgs_generate_messages_cpp)
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(state_estimator_msgs_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/state_estimator_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/state_estimator_msgs
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(state_estimator_msgs_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()
if(TARGET sensor_msgs_generate_messages_eus)
  add_dependencies(state_estimator_msgs_generate_messages_eus sensor_msgs_generate_messages_eus)
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(state_estimator_msgs_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/state_estimator_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/state_estimator_msgs
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(state_estimator_msgs_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()
if(TARGET sensor_msgs_generate_messages_lisp)
  add_dependencies(state_estimator_msgs_generate_messages_lisp sensor_msgs_generate_messages_lisp)
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(state_estimator_msgs_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/state_estimator_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/state_estimator_msgs
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(state_estimator_msgs_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()
if(TARGET sensor_msgs_generate_messages_nodejs)
  add_dependencies(state_estimator_msgs_generate_messages_nodejs sensor_msgs_generate_messages_nodejs)
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(state_estimator_msgs_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/state_estimator_msgs)
  install(CODE "execute_process(COMMAND \"/home/ynistico-iit.local/miniforge3/envs/muse/bin/python3.11\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/state_estimator_msgs\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/state_estimator_msgs
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(state_estimator_msgs_generate_messages_py geometry_msgs_generate_messages_py)
endif()
if(TARGET sensor_msgs_generate_messages_py)
  add_dependencies(state_estimator_msgs_generate_messages_py sensor_msgs_generate_messages_py)
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(state_estimator_msgs_generate_messages_py std_msgs_generate_messages_py)
endif()
