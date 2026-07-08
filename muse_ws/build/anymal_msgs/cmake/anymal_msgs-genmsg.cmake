# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "anymal_msgs: 3 messages, 0 services")

set(MSG_I_FLAGS "-Ianymal_msgs:/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/anymal_msgs/msg;-Igeometry_msgs:/home/ynistico-iit.local/miniforge3/envs/muse/share/geometry_msgs/cmake/../msg;-Istd_msgs:/home/ynistico-iit.local/miniforge3/envs/muse/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(anymal_msgs_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/anymal_msgs/msg/AnymalState.msg" NAME_WE)
add_custom_target(_anymal_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "anymal_msgs" "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/anymal_msgs/msg/AnymalState.msg" "geometry_msgs/Vector3:geometry_msgs/TwistStamped:geometry_msgs/Quaternion:geometry_msgs/Transform:std_msgs/Header:geometry_msgs/Twist:geometry_msgs/Point:anymal_msgs/Contact:anymal_msgs/ExtendedJointState:geometry_msgs/Pose:geometry_msgs/Wrench:geometry_msgs/TransformStamped:geometry_msgs/PoseStamped"
)

get_filename_component(_filename "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/anymal_msgs/msg/Contact.msg" NAME_WE)
add_custom_target(_anymal_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "anymal_msgs" "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/anymal_msgs/msg/Contact.msg" "std_msgs/Header:geometry_msgs/Wrench:geometry_msgs/Vector3:geometry_msgs/Point"
)

get_filename_component(_filename "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/anymal_msgs/msg/ExtendedJointState.msg" NAME_WE)
add_custom_target(_anymal_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "anymal_msgs" "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/anymal_msgs/msg/ExtendedJointState.msg" "std_msgs/Header"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(anymal_msgs
  "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/anymal_msgs/msg/AnymalState.msg"
  "${MSG_I_FLAGS}"
  "/home/ynistico-iit.local/miniforge3/envs/muse/share/geometry_msgs/cmake/../msg/Vector3.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/geometry_msgs/cmake/../msg/TwistStamped.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/geometry_msgs/cmake/../msg/Quaternion.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/geometry_msgs/cmake/../msg/Transform.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/std_msgs/cmake/../msg/Header.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/geometry_msgs/cmake/../msg/Twist.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/geometry_msgs/cmake/../msg/Point.msg;/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/anymal_msgs/msg/Contact.msg;/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/anymal_msgs/msg/ExtendedJointState.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/geometry_msgs/cmake/../msg/Pose.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/geometry_msgs/cmake/../msg/Wrench.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/geometry_msgs/cmake/../msg/TransformStamped.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/geometry_msgs/cmake/../msg/PoseStamped.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/anymal_msgs
)
_generate_msg_cpp(anymal_msgs
  "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/anymal_msgs/msg/Contact.msg"
  "${MSG_I_FLAGS}"
  "/home/ynistico-iit.local/miniforge3/envs/muse/share/std_msgs/cmake/../msg/Header.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/geometry_msgs/cmake/../msg/Wrench.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/geometry_msgs/cmake/../msg/Vector3.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/anymal_msgs
)
_generate_msg_cpp(anymal_msgs
  "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/anymal_msgs/msg/ExtendedJointState.msg"
  "${MSG_I_FLAGS}"
  "/home/ynistico-iit.local/miniforge3/envs/muse/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/anymal_msgs
)

### Generating Services

### Generating Module File
_generate_module_cpp(anymal_msgs
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/anymal_msgs
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(anymal_msgs_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(anymal_msgs_generate_messages anymal_msgs_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/anymal_msgs/msg/AnymalState.msg" NAME_WE)
add_dependencies(anymal_msgs_generate_messages_cpp _anymal_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/anymal_msgs/msg/Contact.msg" NAME_WE)
add_dependencies(anymal_msgs_generate_messages_cpp _anymal_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/anymal_msgs/msg/ExtendedJointState.msg" NAME_WE)
add_dependencies(anymal_msgs_generate_messages_cpp _anymal_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(anymal_msgs_gencpp)
add_dependencies(anymal_msgs_gencpp anymal_msgs_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS anymal_msgs_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(anymal_msgs
  "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/anymal_msgs/msg/AnymalState.msg"
  "${MSG_I_FLAGS}"
  "/home/ynistico-iit.local/miniforge3/envs/muse/share/geometry_msgs/cmake/../msg/Vector3.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/geometry_msgs/cmake/../msg/TwistStamped.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/geometry_msgs/cmake/../msg/Quaternion.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/geometry_msgs/cmake/../msg/Transform.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/std_msgs/cmake/../msg/Header.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/geometry_msgs/cmake/../msg/Twist.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/geometry_msgs/cmake/../msg/Point.msg;/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/anymal_msgs/msg/Contact.msg;/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/anymal_msgs/msg/ExtendedJointState.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/geometry_msgs/cmake/../msg/Pose.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/geometry_msgs/cmake/../msg/Wrench.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/geometry_msgs/cmake/../msg/TransformStamped.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/geometry_msgs/cmake/../msg/PoseStamped.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/anymal_msgs
)
_generate_msg_eus(anymal_msgs
  "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/anymal_msgs/msg/Contact.msg"
  "${MSG_I_FLAGS}"
  "/home/ynistico-iit.local/miniforge3/envs/muse/share/std_msgs/cmake/../msg/Header.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/geometry_msgs/cmake/../msg/Wrench.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/geometry_msgs/cmake/../msg/Vector3.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/anymal_msgs
)
_generate_msg_eus(anymal_msgs
  "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/anymal_msgs/msg/ExtendedJointState.msg"
  "${MSG_I_FLAGS}"
  "/home/ynistico-iit.local/miniforge3/envs/muse/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/anymal_msgs
)

### Generating Services

### Generating Module File
_generate_module_eus(anymal_msgs
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/anymal_msgs
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(anymal_msgs_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(anymal_msgs_generate_messages anymal_msgs_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/anymal_msgs/msg/AnymalState.msg" NAME_WE)
add_dependencies(anymal_msgs_generate_messages_eus _anymal_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/anymal_msgs/msg/Contact.msg" NAME_WE)
add_dependencies(anymal_msgs_generate_messages_eus _anymal_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/anymal_msgs/msg/ExtendedJointState.msg" NAME_WE)
add_dependencies(anymal_msgs_generate_messages_eus _anymal_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(anymal_msgs_geneus)
add_dependencies(anymal_msgs_geneus anymal_msgs_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS anymal_msgs_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(anymal_msgs
  "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/anymal_msgs/msg/AnymalState.msg"
  "${MSG_I_FLAGS}"
  "/home/ynistico-iit.local/miniforge3/envs/muse/share/geometry_msgs/cmake/../msg/Vector3.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/geometry_msgs/cmake/../msg/TwistStamped.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/geometry_msgs/cmake/../msg/Quaternion.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/geometry_msgs/cmake/../msg/Transform.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/std_msgs/cmake/../msg/Header.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/geometry_msgs/cmake/../msg/Twist.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/geometry_msgs/cmake/../msg/Point.msg;/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/anymal_msgs/msg/Contact.msg;/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/anymal_msgs/msg/ExtendedJointState.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/geometry_msgs/cmake/../msg/Pose.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/geometry_msgs/cmake/../msg/Wrench.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/geometry_msgs/cmake/../msg/TransformStamped.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/geometry_msgs/cmake/../msg/PoseStamped.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/anymal_msgs
)
_generate_msg_lisp(anymal_msgs
  "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/anymal_msgs/msg/Contact.msg"
  "${MSG_I_FLAGS}"
  "/home/ynistico-iit.local/miniforge3/envs/muse/share/std_msgs/cmake/../msg/Header.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/geometry_msgs/cmake/../msg/Wrench.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/geometry_msgs/cmake/../msg/Vector3.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/anymal_msgs
)
_generate_msg_lisp(anymal_msgs
  "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/anymal_msgs/msg/ExtendedJointState.msg"
  "${MSG_I_FLAGS}"
  "/home/ynistico-iit.local/miniforge3/envs/muse/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/anymal_msgs
)

### Generating Services

### Generating Module File
_generate_module_lisp(anymal_msgs
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/anymal_msgs
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(anymal_msgs_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(anymal_msgs_generate_messages anymal_msgs_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/anymal_msgs/msg/AnymalState.msg" NAME_WE)
add_dependencies(anymal_msgs_generate_messages_lisp _anymal_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/anymal_msgs/msg/Contact.msg" NAME_WE)
add_dependencies(anymal_msgs_generate_messages_lisp _anymal_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/anymal_msgs/msg/ExtendedJointState.msg" NAME_WE)
add_dependencies(anymal_msgs_generate_messages_lisp _anymal_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(anymal_msgs_genlisp)
add_dependencies(anymal_msgs_genlisp anymal_msgs_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS anymal_msgs_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(anymal_msgs
  "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/anymal_msgs/msg/AnymalState.msg"
  "${MSG_I_FLAGS}"
  "/home/ynistico-iit.local/miniforge3/envs/muse/share/geometry_msgs/cmake/../msg/Vector3.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/geometry_msgs/cmake/../msg/TwistStamped.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/geometry_msgs/cmake/../msg/Quaternion.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/geometry_msgs/cmake/../msg/Transform.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/std_msgs/cmake/../msg/Header.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/geometry_msgs/cmake/../msg/Twist.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/geometry_msgs/cmake/../msg/Point.msg;/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/anymal_msgs/msg/Contact.msg;/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/anymal_msgs/msg/ExtendedJointState.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/geometry_msgs/cmake/../msg/Pose.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/geometry_msgs/cmake/../msg/Wrench.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/geometry_msgs/cmake/../msg/TransformStamped.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/geometry_msgs/cmake/../msg/PoseStamped.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/anymal_msgs
)
_generate_msg_nodejs(anymal_msgs
  "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/anymal_msgs/msg/Contact.msg"
  "${MSG_I_FLAGS}"
  "/home/ynistico-iit.local/miniforge3/envs/muse/share/std_msgs/cmake/../msg/Header.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/geometry_msgs/cmake/../msg/Wrench.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/geometry_msgs/cmake/../msg/Vector3.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/anymal_msgs
)
_generate_msg_nodejs(anymal_msgs
  "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/anymal_msgs/msg/ExtendedJointState.msg"
  "${MSG_I_FLAGS}"
  "/home/ynistico-iit.local/miniforge3/envs/muse/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/anymal_msgs
)

### Generating Services

### Generating Module File
_generate_module_nodejs(anymal_msgs
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/anymal_msgs
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(anymal_msgs_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(anymal_msgs_generate_messages anymal_msgs_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/anymal_msgs/msg/AnymalState.msg" NAME_WE)
add_dependencies(anymal_msgs_generate_messages_nodejs _anymal_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/anymal_msgs/msg/Contact.msg" NAME_WE)
add_dependencies(anymal_msgs_generate_messages_nodejs _anymal_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/anymal_msgs/msg/ExtendedJointState.msg" NAME_WE)
add_dependencies(anymal_msgs_generate_messages_nodejs _anymal_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(anymal_msgs_gennodejs)
add_dependencies(anymal_msgs_gennodejs anymal_msgs_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS anymal_msgs_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(anymal_msgs
  "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/anymal_msgs/msg/AnymalState.msg"
  "${MSG_I_FLAGS}"
  "/home/ynistico-iit.local/miniforge3/envs/muse/share/geometry_msgs/cmake/../msg/Vector3.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/geometry_msgs/cmake/../msg/TwistStamped.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/geometry_msgs/cmake/../msg/Quaternion.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/geometry_msgs/cmake/../msg/Transform.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/std_msgs/cmake/../msg/Header.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/geometry_msgs/cmake/../msg/Twist.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/geometry_msgs/cmake/../msg/Point.msg;/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/anymal_msgs/msg/Contact.msg;/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/anymal_msgs/msg/ExtendedJointState.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/geometry_msgs/cmake/../msg/Pose.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/geometry_msgs/cmake/../msg/Wrench.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/geometry_msgs/cmake/../msg/TransformStamped.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/geometry_msgs/cmake/../msg/PoseStamped.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/anymal_msgs
)
_generate_msg_py(anymal_msgs
  "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/anymal_msgs/msg/Contact.msg"
  "${MSG_I_FLAGS}"
  "/home/ynistico-iit.local/miniforge3/envs/muse/share/std_msgs/cmake/../msg/Header.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/geometry_msgs/cmake/../msg/Wrench.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/geometry_msgs/cmake/../msg/Vector3.msg;/home/ynistico-iit.local/miniforge3/envs/muse/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/anymal_msgs
)
_generate_msg_py(anymal_msgs
  "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/anymal_msgs/msg/ExtendedJointState.msg"
  "${MSG_I_FLAGS}"
  "/home/ynistico-iit.local/miniforge3/envs/muse/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/anymal_msgs
)

### Generating Services

### Generating Module File
_generate_module_py(anymal_msgs
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/anymal_msgs
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(anymal_msgs_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(anymal_msgs_generate_messages anymal_msgs_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/anymal_msgs/msg/AnymalState.msg" NAME_WE)
add_dependencies(anymal_msgs_generate_messages_py _anymal_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/anymal_msgs/msg/Contact.msg" NAME_WE)
add_dependencies(anymal_msgs_generate_messages_py _anymal_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/anymal_msgs/msg/ExtendedJointState.msg" NAME_WE)
add_dependencies(anymal_msgs_generate_messages_py _anymal_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(anymal_msgs_genpy)
add_dependencies(anymal_msgs_genpy anymal_msgs_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS anymal_msgs_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/anymal_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/anymal_msgs
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(anymal_msgs_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(anymal_msgs_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/anymal_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/anymal_msgs
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(anymal_msgs_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(anymal_msgs_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/anymal_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/anymal_msgs
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(anymal_msgs_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(anymal_msgs_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/anymal_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/anymal_msgs
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(anymal_msgs_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(anymal_msgs_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/anymal_msgs)
  install(CODE "execute_process(COMMAND \"/home/ynistico-iit.local/miniforge3/envs/muse/bin/python3.11\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/anymal_msgs\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/anymal_msgs
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(anymal_msgs_generate_messages_py geometry_msgs_generate_messages_py)
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(anymal_msgs_generate_messages_py std_msgs_generate_messages_py)
endif()
