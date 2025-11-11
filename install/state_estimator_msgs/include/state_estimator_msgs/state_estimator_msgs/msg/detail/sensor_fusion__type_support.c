// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from state_estimator_msgs:msg/SensorFusion.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "state_estimator_msgs/msg/detail/sensor_fusion__rosidl_typesupport_introspection_c.h"
#include "state_estimator_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "state_estimator_msgs/msg/detail/sensor_fusion__functions.h"
#include "state_estimator_msgs/msg/detail/sensor_fusion__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void state_estimator_msgs__msg__SensorFusion__rosidl_typesupport_introspection_c__SensorFusion_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  state_estimator_msgs__msg__SensorFusion__init(message_memory);
}

void state_estimator_msgs__msg__SensorFusion__rosidl_typesupport_introspection_c__SensorFusion_fini_function(void * message_memory)
{
  state_estimator_msgs__msg__SensorFusion__fini(message_memory);
}

size_t state_estimator_msgs__msg__SensorFusion__rosidl_typesupport_introspection_c__size_function__SensorFusion__position(
  const void * untyped_member)
{
  (void)untyped_member;
  return 3;
}

const void * state_estimator_msgs__msg__SensorFusion__rosidl_typesupport_introspection_c__get_const_function__SensorFusion__position(
  const void * untyped_member, size_t index)
{
  const double * member =
    (const double *)(untyped_member);
  return &member[index];
}

void * state_estimator_msgs__msg__SensorFusion__rosidl_typesupport_introspection_c__get_function__SensorFusion__position(
  void * untyped_member, size_t index)
{
  double * member =
    (double *)(untyped_member);
  return &member[index];
}

void state_estimator_msgs__msg__SensorFusion__rosidl_typesupport_introspection_c__fetch_function__SensorFusion__position(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    state_estimator_msgs__msg__SensorFusion__rosidl_typesupport_introspection_c__get_const_function__SensorFusion__position(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void state_estimator_msgs__msg__SensorFusion__rosidl_typesupport_introspection_c__assign_function__SensorFusion__position(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    state_estimator_msgs__msg__SensorFusion__rosidl_typesupport_introspection_c__get_function__SensorFusion__position(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

size_t state_estimator_msgs__msg__SensorFusion__rosidl_typesupport_introspection_c__size_function__SensorFusion__linear_velocity(
  const void * untyped_member)
{
  (void)untyped_member;
  return 3;
}

const void * state_estimator_msgs__msg__SensorFusion__rosidl_typesupport_introspection_c__get_const_function__SensorFusion__linear_velocity(
  const void * untyped_member, size_t index)
{
  const double * member =
    (const double *)(untyped_member);
  return &member[index];
}

void * state_estimator_msgs__msg__SensorFusion__rosidl_typesupport_introspection_c__get_function__SensorFusion__linear_velocity(
  void * untyped_member, size_t index)
{
  double * member =
    (double *)(untyped_member);
  return &member[index];
}

void state_estimator_msgs__msg__SensorFusion__rosidl_typesupport_introspection_c__fetch_function__SensorFusion__linear_velocity(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    state_estimator_msgs__msg__SensorFusion__rosidl_typesupport_introspection_c__get_const_function__SensorFusion__linear_velocity(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void state_estimator_msgs__msg__SensorFusion__rosidl_typesupport_introspection_c__assign_function__SensorFusion__linear_velocity(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    state_estimator_msgs__msg__SensorFusion__rosidl_typesupport_introspection_c__get_function__SensorFusion__linear_velocity(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

static rosidl_typesupport_introspection_c__MessageMember state_estimator_msgs__msg__SensorFusion__rosidl_typesupport_introspection_c__SensorFusion_message_member_array[3] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(state_estimator_msgs__msg__SensorFusion, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "position",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    true,  // is array
    3,  // array size
    false,  // is upper bound
    offsetof(state_estimator_msgs__msg__SensorFusion, position),  // bytes offset in struct
    NULL,  // default value
    state_estimator_msgs__msg__SensorFusion__rosidl_typesupport_introspection_c__size_function__SensorFusion__position,  // size() function pointer
    state_estimator_msgs__msg__SensorFusion__rosidl_typesupport_introspection_c__get_const_function__SensorFusion__position,  // get_const(index) function pointer
    state_estimator_msgs__msg__SensorFusion__rosidl_typesupport_introspection_c__get_function__SensorFusion__position,  // get(index) function pointer
    state_estimator_msgs__msg__SensorFusion__rosidl_typesupport_introspection_c__fetch_function__SensorFusion__position,  // fetch(index, &value) function pointer
    state_estimator_msgs__msg__SensorFusion__rosidl_typesupport_introspection_c__assign_function__SensorFusion__position,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "linear_velocity",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    true,  // is array
    3,  // array size
    false,  // is upper bound
    offsetof(state_estimator_msgs__msg__SensorFusion, linear_velocity),  // bytes offset in struct
    NULL,  // default value
    state_estimator_msgs__msg__SensorFusion__rosidl_typesupport_introspection_c__size_function__SensorFusion__linear_velocity,  // size() function pointer
    state_estimator_msgs__msg__SensorFusion__rosidl_typesupport_introspection_c__get_const_function__SensorFusion__linear_velocity,  // get_const(index) function pointer
    state_estimator_msgs__msg__SensorFusion__rosidl_typesupport_introspection_c__get_function__SensorFusion__linear_velocity,  // get(index) function pointer
    state_estimator_msgs__msg__SensorFusion__rosidl_typesupport_introspection_c__fetch_function__SensorFusion__linear_velocity,  // fetch(index, &value) function pointer
    state_estimator_msgs__msg__SensorFusion__rosidl_typesupport_introspection_c__assign_function__SensorFusion__linear_velocity,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers state_estimator_msgs__msg__SensorFusion__rosidl_typesupport_introspection_c__SensorFusion_message_members = {
  "state_estimator_msgs__msg",  // message namespace
  "SensorFusion",  // message name
  3,  // number of fields
  sizeof(state_estimator_msgs__msg__SensorFusion),
  false,  // has_any_key_member_
  state_estimator_msgs__msg__SensorFusion__rosidl_typesupport_introspection_c__SensorFusion_message_member_array,  // message members
  state_estimator_msgs__msg__SensorFusion__rosidl_typesupport_introspection_c__SensorFusion_init_function,  // function to initialize message memory (memory has to be allocated)
  state_estimator_msgs__msg__SensorFusion__rosidl_typesupport_introspection_c__SensorFusion_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t state_estimator_msgs__msg__SensorFusion__rosidl_typesupport_introspection_c__SensorFusion_message_type_support_handle = {
  0,
  &state_estimator_msgs__msg__SensorFusion__rosidl_typesupport_introspection_c__SensorFusion_message_members,
  get_message_typesupport_handle_function,
  &state_estimator_msgs__msg__SensorFusion__get_type_hash,
  &state_estimator_msgs__msg__SensorFusion__get_type_description,
  &state_estimator_msgs__msg__SensorFusion__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_state_estimator_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, state_estimator_msgs, msg, SensorFusion)() {
  state_estimator_msgs__msg__SensorFusion__rosidl_typesupport_introspection_c__SensorFusion_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  if (!state_estimator_msgs__msg__SensorFusion__rosidl_typesupport_introspection_c__SensorFusion_message_type_support_handle.typesupport_identifier) {
    state_estimator_msgs__msg__SensorFusion__rosidl_typesupport_introspection_c__SensorFusion_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &state_estimator_msgs__msg__SensorFusion__rosidl_typesupport_introspection_c__SensorFusion_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
