// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from state_estimator_msgs:msg/Attitude.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "state_estimator_msgs/msg/detail/attitude__rosidl_typesupport_introspection_c.h"
#include "state_estimator_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "state_estimator_msgs/msg/detail/attitude__functions.h"
#include "state_estimator_msgs/msg/detail/attitude__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void state_estimator_msgs__msg__Attitude__rosidl_typesupport_introspection_c__Attitude_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  state_estimator_msgs__msg__Attitude__init(message_memory);
}

void state_estimator_msgs__msg__Attitude__rosidl_typesupport_introspection_c__Attitude_fini_function(void * message_memory)
{
  state_estimator_msgs__msg__Attitude__fini(message_memory);
}

size_t state_estimator_msgs__msg__Attitude__rosidl_typesupport_introspection_c__size_function__Attitude__quaternion(
  const void * untyped_member)
{
  (void)untyped_member;
  return 4;
}

const void * state_estimator_msgs__msg__Attitude__rosidl_typesupport_introspection_c__get_const_function__Attitude__quaternion(
  const void * untyped_member, size_t index)
{
  const double * member =
    (const double *)(untyped_member);
  return &member[index];
}

void * state_estimator_msgs__msg__Attitude__rosidl_typesupport_introspection_c__get_function__Attitude__quaternion(
  void * untyped_member, size_t index)
{
  double * member =
    (double *)(untyped_member);
  return &member[index];
}

void state_estimator_msgs__msg__Attitude__rosidl_typesupport_introspection_c__fetch_function__Attitude__quaternion(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    state_estimator_msgs__msg__Attitude__rosidl_typesupport_introspection_c__get_const_function__Attitude__quaternion(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void state_estimator_msgs__msg__Attitude__rosidl_typesupport_introspection_c__assign_function__Attitude__quaternion(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    state_estimator_msgs__msg__Attitude__rosidl_typesupport_introspection_c__get_function__Attitude__quaternion(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

size_t state_estimator_msgs__msg__Attitude__rosidl_typesupport_introspection_c__size_function__Attitude__angular_velocity(
  const void * untyped_member)
{
  (void)untyped_member;
  return 3;
}

const void * state_estimator_msgs__msg__Attitude__rosidl_typesupport_introspection_c__get_const_function__Attitude__angular_velocity(
  const void * untyped_member, size_t index)
{
  const double * member =
    (const double *)(untyped_member);
  return &member[index];
}

void * state_estimator_msgs__msg__Attitude__rosidl_typesupport_introspection_c__get_function__Attitude__angular_velocity(
  void * untyped_member, size_t index)
{
  double * member =
    (double *)(untyped_member);
  return &member[index];
}

void state_estimator_msgs__msg__Attitude__rosidl_typesupport_introspection_c__fetch_function__Attitude__angular_velocity(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    state_estimator_msgs__msg__Attitude__rosidl_typesupport_introspection_c__get_const_function__Attitude__angular_velocity(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void state_estimator_msgs__msg__Attitude__rosidl_typesupport_introspection_c__assign_function__Attitude__angular_velocity(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    state_estimator_msgs__msg__Attitude__rosidl_typesupport_introspection_c__get_function__Attitude__angular_velocity(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

static rosidl_typesupport_introspection_c__MessageMember state_estimator_msgs__msg__Attitude__rosidl_typesupport_introspection_c__Attitude_message_member_array[6] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(state_estimator_msgs__msg__Attitude, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "quaternion",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    true,  // is array
    4,  // array size
    false,  // is upper bound
    offsetof(state_estimator_msgs__msg__Attitude, quaternion),  // bytes offset in struct
    NULL,  // default value
    state_estimator_msgs__msg__Attitude__rosidl_typesupport_introspection_c__size_function__Attitude__quaternion,  // size() function pointer
    state_estimator_msgs__msg__Attitude__rosidl_typesupport_introspection_c__get_const_function__Attitude__quaternion,  // get_const(index) function pointer
    state_estimator_msgs__msg__Attitude__rosidl_typesupport_introspection_c__get_function__Attitude__quaternion,  // get(index) function pointer
    state_estimator_msgs__msg__Attitude__rosidl_typesupport_introspection_c__fetch_function__Attitude__quaternion,  // fetch(index, &value) function pointer
    state_estimator_msgs__msg__Attitude__rosidl_typesupport_introspection_c__assign_function__Attitude__quaternion,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "roll_deg",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(state_estimator_msgs__msg__Attitude, roll_deg),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "pitch_deg",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(state_estimator_msgs__msg__Attitude, pitch_deg),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "yaw_deg",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(state_estimator_msgs__msg__Attitude, yaw_deg),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "angular_velocity",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    true,  // is array
    3,  // array size
    false,  // is upper bound
    offsetof(state_estimator_msgs__msg__Attitude, angular_velocity),  // bytes offset in struct
    NULL,  // default value
    state_estimator_msgs__msg__Attitude__rosidl_typesupport_introspection_c__size_function__Attitude__angular_velocity,  // size() function pointer
    state_estimator_msgs__msg__Attitude__rosidl_typesupport_introspection_c__get_const_function__Attitude__angular_velocity,  // get_const(index) function pointer
    state_estimator_msgs__msg__Attitude__rosidl_typesupport_introspection_c__get_function__Attitude__angular_velocity,  // get(index) function pointer
    state_estimator_msgs__msg__Attitude__rosidl_typesupport_introspection_c__fetch_function__Attitude__angular_velocity,  // fetch(index, &value) function pointer
    state_estimator_msgs__msg__Attitude__rosidl_typesupport_introspection_c__assign_function__Attitude__angular_velocity,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers state_estimator_msgs__msg__Attitude__rosidl_typesupport_introspection_c__Attitude_message_members = {
  "state_estimator_msgs__msg",  // message namespace
  "Attitude",  // message name
  6,  // number of fields
  sizeof(state_estimator_msgs__msg__Attitude),
  false,  // has_any_key_member_
  state_estimator_msgs__msg__Attitude__rosidl_typesupport_introspection_c__Attitude_message_member_array,  // message members
  state_estimator_msgs__msg__Attitude__rosidl_typesupport_introspection_c__Attitude_init_function,  // function to initialize message memory (memory has to be allocated)
  state_estimator_msgs__msg__Attitude__rosidl_typesupport_introspection_c__Attitude_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t state_estimator_msgs__msg__Attitude__rosidl_typesupport_introspection_c__Attitude_message_type_support_handle = {
  0,
  &state_estimator_msgs__msg__Attitude__rosidl_typesupport_introspection_c__Attitude_message_members,
  get_message_typesupport_handle_function,
  &state_estimator_msgs__msg__Attitude__get_type_hash,
  &state_estimator_msgs__msg__Attitude__get_type_description,
  &state_estimator_msgs__msg__Attitude__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_state_estimator_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, state_estimator_msgs, msg, Attitude)() {
  state_estimator_msgs__msg__Attitude__rosidl_typesupport_introspection_c__Attitude_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  if (!state_estimator_msgs__msg__Attitude__rosidl_typesupport_introspection_c__Attitude_message_type_support_handle.typesupport_identifier) {
    state_estimator_msgs__msg__Attitude__rosidl_typesupport_introspection_c__Attitude_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &state_estimator_msgs__msg__Attitude__rosidl_typesupport_introspection_c__Attitude_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
