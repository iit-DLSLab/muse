// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from state_estimator_msgs:msg/LegOdometry.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "state_estimator_msgs/msg/detail/leg_odometry__rosidl_typesupport_introspection_c.h"
#include "state_estimator_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "state_estimator_msgs/msg/detail/leg_odometry__functions.h"
#include "state_estimator_msgs/msg/detail/leg_odometry__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void state_estimator_msgs__msg__LegOdometry__rosidl_typesupport_introspection_c__LegOdometry_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  state_estimator_msgs__msg__LegOdometry__init(message_memory);
}

void state_estimator_msgs__msg__LegOdometry__rosidl_typesupport_introspection_c__LegOdometry_fini_function(void * message_memory)
{
  state_estimator_msgs__msg__LegOdometry__fini(message_memory);
}

size_t state_estimator_msgs__msg__LegOdometry__rosidl_typesupport_introspection_c__size_function__LegOdometry__lin_vel_lf(
  const void * untyped_member)
{
  (void)untyped_member;
  return 3;
}

const void * state_estimator_msgs__msg__LegOdometry__rosidl_typesupport_introspection_c__get_const_function__LegOdometry__lin_vel_lf(
  const void * untyped_member, size_t index)
{
  const double * member =
    (const double *)(untyped_member);
  return &member[index];
}

void * state_estimator_msgs__msg__LegOdometry__rosidl_typesupport_introspection_c__get_function__LegOdometry__lin_vel_lf(
  void * untyped_member, size_t index)
{
  double * member =
    (double *)(untyped_member);
  return &member[index];
}

void state_estimator_msgs__msg__LegOdometry__rosidl_typesupport_introspection_c__fetch_function__LegOdometry__lin_vel_lf(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    state_estimator_msgs__msg__LegOdometry__rosidl_typesupport_introspection_c__get_const_function__LegOdometry__lin_vel_lf(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void state_estimator_msgs__msg__LegOdometry__rosidl_typesupport_introspection_c__assign_function__LegOdometry__lin_vel_lf(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    state_estimator_msgs__msg__LegOdometry__rosidl_typesupport_introspection_c__get_function__LegOdometry__lin_vel_lf(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

size_t state_estimator_msgs__msg__LegOdometry__rosidl_typesupport_introspection_c__size_function__LegOdometry__lin_vel_rf(
  const void * untyped_member)
{
  (void)untyped_member;
  return 3;
}

const void * state_estimator_msgs__msg__LegOdometry__rosidl_typesupport_introspection_c__get_const_function__LegOdometry__lin_vel_rf(
  const void * untyped_member, size_t index)
{
  const double * member =
    (const double *)(untyped_member);
  return &member[index];
}

void * state_estimator_msgs__msg__LegOdometry__rosidl_typesupport_introspection_c__get_function__LegOdometry__lin_vel_rf(
  void * untyped_member, size_t index)
{
  double * member =
    (double *)(untyped_member);
  return &member[index];
}

void state_estimator_msgs__msg__LegOdometry__rosidl_typesupport_introspection_c__fetch_function__LegOdometry__lin_vel_rf(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    state_estimator_msgs__msg__LegOdometry__rosidl_typesupport_introspection_c__get_const_function__LegOdometry__lin_vel_rf(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void state_estimator_msgs__msg__LegOdometry__rosidl_typesupport_introspection_c__assign_function__LegOdometry__lin_vel_rf(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    state_estimator_msgs__msg__LegOdometry__rosidl_typesupport_introspection_c__get_function__LegOdometry__lin_vel_rf(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

size_t state_estimator_msgs__msg__LegOdometry__rosidl_typesupport_introspection_c__size_function__LegOdometry__lin_vel_lh(
  const void * untyped_member)
{
  (void)untyped_member;
  return 3;
}

const void * state_estimator_msgs__msg__LegOdometry__rosidl_typesupport_introspection_c__get_const_function__LegOdometry__lin_vel_lh(
  const void * untyped_member, size_t index)
{
  const double * member =
    (const double *)(untyped_member);
  return &member[index];
}

void * state_estimator_msgs__msg__LegOdometry__rosidl_typesupport_introspection_c__get_function__LegOdometry__lin_vel_lh(
  void * untyped_member, size_t index)
{
  double * member =
    (double *)(untyped_member);
  return &member[index];
}

void state_estimator_msgs__msg__LegOdometry__rosidl_typesupport_introspection_c__fetch_function__LegOdometry__lin_vel_lh(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    state_estimator_msgs__msg__LegOdometry__rosidl_typesupport_introspection_c__get_const_function__LegOdometry__lin_vel_lh(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void state_estimator_msgs__msg__LegOdometry__rosidl_typesupport_introspection_c__assign_function__LegOdometry__lin_vel_lh(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    state_estimator_msgs__msg__LegOdometry__rosidl_typesupport_introspection_c__get_function__LegOdometry__lin_vel_lh(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

size_t state_estimator_msgs__msg__LegOdometry__rosidl_typesupport_introspection_c__size_function__LegOdometry__lin_vel_rh(
  const void * untyped_member)
{
  (void)untyped_member;
  return 3;
}

const void * state_estimator_msgs__msg__LegOdometry__rosidl_typesupport_introspection_c__get_const_function__LegOdometry__lin_vel_rh(
  const void * untyped_member, size_t index)
{
  const double * member =
    (const double *)(untyped_member);
  return &member[index];
}

void * state_estimator_msgs__msg__LegOdometry__rosidl_typesupport_introspection_c__get_function__LegOdometry__lin_vel_rh(
  void * untyped_member, size_t index)
{
  double * member =
    (double *)(untyped_member);
  return &member[index];
}

void state_estimator_msgs__msg__LegOdometry__rosidl_typesupport_introspection_c__fetch_function__LegOdometry__lin_vel_rh(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    state_estimator_msgs__msg__LegOdometry__rosidl_typesupport_introspection_c__get_const_function__LegOdometry__lin_vel_rh(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void state_estimator_msgs__msg__LegOdometry__rosidl_typesupport_introspection_c__assign_function__LegOdometry__lin_vel_rh(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    state_estimator_msgs__msg__LegOdometry__rosidl_typesupport_introspection_c__get_function__LegOdometry__lin_vel_rh(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

size_t state_estimator_msgs__msg__LegOdometry__rosidl_typesupport_introspection_c__size_function__LegOdometry__base_velocity(
  const void * untyped_member)
{
  (void)untyped_member;
  return 3;
}

const void * state_estimator_msgs__msg__LegOdometry__rosidl_typesupport_introspection_c__get_const_function__LegOdometry__base_velocity(
  const void * untyped_member, size_t index)
{
  const double * member =
    (const double *)(untyped_member);
  return &member[index];
}

void * state_estimator_msgs__msg__LegOdometry__rosidl_typesupport_introspection_c__get_function__LegOdometry__base_velocity(
  void * untyped_member, size_t index)
{
  double * member =
    (double *)(untyped_member);
  return &member[index];
}

void state_estimator_msgs__msg__LegOdometry__rosidl_typesupport_introspection_c__fetch_function__LegOdometry__base_velocity(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    state_estimator_msgs__msg__LegOdometry__rosidl_typesupport_introspection_c__get_const_function__LegOdometry__base_velocity(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void state_estimator_msgs__msg__LegOdometry__rosidl_typesupport_introspection_c__assign_function__LegOdometry__base_velocity(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    state_estimator_msgs__msg__LegOdometry__rosidl_typesupport_introspection_c__get_function__LegOdometry__base_velocity(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

static rosidl_typesupport_introspection_c__MessageMember state_estimator_msgs__msg__LegOdometry__rosidl_typesupport_introspection_c__LegOdometry_message_member_array[6] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(state_estimator_msgs__msg__LegOdometry, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "lin_vel_lf",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    true,  // is array
    3,  // array size
    false,  // is upper bound
    offsetof(state_estimator_msgs__msg__LegOdometry, lin_vel_lf),  // bytes offset in struct
    NULL,  // default value
    state_estimator_msgs__msg__LegOdometry__rosidl_typesupport_introspection_c__size_function__LegOdometry__lin_vel_lf,  // size() function pointer
    state_estimator_msgs__msg__LegOdometry__rosidl_typesupport_introspection_c__get_const_function__LegOdometry__lin_vel_lf,  // get_const(index) function pointer
    state_estimator_msgs__msg__LegOdometry__rosidl_typesupport_introspection_c__get_function__LegOdometry__lin_vel_lf,  // get(index) function pointer
    state_estimator_msgs__msg__LegOdometry__rosidl_typesupport_introspection_c__fetch_function__LegOdometry__lin_vel_lf,  // fetch(index, &value) function pointer
    state_estimator_msgs__msg__LegOdometry__rosidl_typesupport_introspection_c__assign_function__LegOdometry__lin_vel_lf,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "lin_vel_rf",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    true,  // is array
    3,  // array size
    false,  // is upper bound
    offsetof(state_estimator_msgs__msg__LegOdometry, lin_vel_rf),  // bytes offset in struct
    NULL,  // default value
    state_estimator_msgs__msg__LegOdometry__rosidl_typesupport_introspection_c__size_function__LegOdometry__lin_vel_rf,  // size() function pointer
    state_estimator_msgs__msg__LegOdometry__rosidl_typesupport_introspection_c__get_const_function__LegOdometry__lin_vel_rf,  // get_const(index) function pointer
    state_estimator_msgs__msg__LegOdometry__rosidl_typesupport_introspection_c__get_function__LegOdometry__lin_vel_rf,  // get(index) function pointer
    state_estimator_msgs__msg__LegOdometry__rosidl_typesupport_introspection_c__fetch_function__LegOdometry__lin_vel_rf,  // fetch(index, &value) function pointer
    state_estimator_msgs__msg__LegOdometry__rosidl_typesupport_introspection_c__assign_function__LegOdometry__lin_vel_rf,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "lin_vel_lh",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    true,  // is array
    3,  // array size
    false,  // is upper bound
    offsetof(state_estimator_msgs__msg__LegOdometry, lin_vel_lh),  // bytes offset in struct
    NULL,  // default value
    state_estimator_msgs__msg__LegOdometry__rosidl_typesupport_introspection_c__size_function__LegOdometry__lin_vel_lh,  // size() function pointer
    state_estimator_msgs__msg__LegOdometry__rosidl_typesupport_introspection_c__get_const_function__LegOdometry__lin_vel_lh,  // get_const(index) function pointer
    state_estimator_msgs__msg__LegOdometry__rosidl_typesupport_introspection_c__get_function__LegOdometry__lin_vel_lh,  // get(index) function pointer
    state_estimator_msgs__msg__LegOdometry__rosidl_typesupport_introspection_c__fetch_function__LegOdometry__lin_vel_lh,  // fetch(index, &value) function pointer
    state_estimator_msgs__msg__LegOdometry__rosidl_typesupport_introspection_c__assign_function__LegOdometry__lin_vel_lh,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "lin_vel_rh",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    true,  // is array
    3,  // array size
    false,  // is upper bound
    offsetof(state_estimator_msgs__msg__LegOdometry, lin_vel_rh),  // bytes offset in struct
    NULL,  // default value
    state_estimator_msgs__msg__LegOdometry__rosidl_typesupport_introspection_c__size_function__LegOdometry__lin_vel_rh,  // size() function pointer
    state_estimator_msgs__msg__LegOdometry__rosidl_typesupport_introspection_c__get_const_function__LegOdometry__lin_vel_rh,  // get_const(index) function pointer
    state_estimator_msgs__msg__LegOdometry__rosidl_typesupport_introspection_c__get_function__LegOdometry__lin_vel_rh,  // get(index) function pointer
    state_estimator_msgs__msg__LegOdometry__rosidl_typesupport_introspection_c__fetch_function__LegOdometry__lin_vel_rh,  // fetch(index, &value) function pointer
    state_estimator_msgs__msg__LegOdometry__rosidl_typesupport_introspection_c__assign_function__LegOdometry__lin_vel_rh,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "base_velocity",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    true,  // is array
    3,  // array size
    false,  // is upper bound
    offsetof(state_estimator_msgs__msg__LegOdometry, base_velocity),  // bytes offset in struct
    NULL,  // default value
    state_estimator_msgs__msg__LegOdometry__rosidl_typesupport_introspection_c__size_function__LegOdometry__base_velocity,  // size() function pointer
    state_estimator_msgs__msg__LegOdometry__rosidl_typesupport_introspection_c__get_const_function__LegOdometry__base_velocity,  // get_const(index) function pointer
    state_estimator_msgs__msg__LegOdometry__rosidl_typesupport_introspection_c__get_function__LegOdometry__base_velocity,  // get(index) function pointer
    state_estimator_msgs__msg__LegOdometry__rosidl_typesupport_introspection_c__fetch_function__LegOdometry__base_velocity,  // fetch(index, &value) function pointer
    state_estimator_msgs__msg__LegOdometry__rosidl_typesupport_introspection_c__assign_function__LegOdometry__base_velocity,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers state_estimator_msgs__msg__LegOdometry__rosidl_typesupport_introspection_c__LegOdometry_message_members = {
  "state_estimator_msgs__msg",  // message namespace
  "LegOdometry",  // message name
  6,  // number of fields
  sizeof(state_estimator_msgs__msg__LegOdometry),
  false,  // has_any_key_member_
  state_estimator_msgs__msg__LegOdometry__rosidl_typesupport_introspection_c__LegOdometry_message_member_array,  // message members
  state_estimator_msgs__msg__LegOdometry__rosidl_typesupport_introspection_c__LegOdometry_init_function,  // function to initialize message memory (memory has to be allocated)
  state_estimator_msgs__msg__LegOdometry__rosidl_typesupport_introspection_c__LegOdometry_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t state_estimator_msgs__msg__LegOdometry__rosidl_typesupport_introspection_c__LegOdometry_message_type_support_handle = {
  0,
  &state_estimator_msgs__msg__LegOdometry__rosidl_typesupport_introspection_c__LegOdometry_message_members,
  get_message_typesupport_handle_function,
  &state_estimator_msgs__msg__LegOdometry__get_type_hash,
  &state_estimator_msgs__msg__LegOdometry__get_type_description,
  &state_estimator_msgs__msg__LegOdometry__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_state_estimator_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, state_estimator_msgs, msg, LegOdometry)() {
  state_estimator_msgs__msg__LegOdometry__rosidl_typesupport_introspection_c__LegOdometry_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  if (!state_estimator_msgs__msg__LegOdometry__rosidl_typesupport_introspection_c__LegOdometry_message_type_support_handle.typesupport_identifier) {
    state_estimator_msgs__msg__LegOdometry__rosidl_typesupport_introspection_c__LegOdometry_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &state_estimator_msgs__msg__LegOdometry__rosidl_typesupport_introspection_c__LegOdometry_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
