// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from state_estimator_msgs:msg/JointStateWithAcceleration.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "state_estimator_msgs/msg/detail/joint_state_with_acceleration__rosidl_typesupport_introspection_c.h"
#include "state_estimator_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "state_estimator_msgs/msg/detail/joint_state_with_acceleration__functions.h"
#include "state_estimator_msgs/msg/detail/joint_state_with_acceleration__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `name`
#include "rosidl_runtime_c/string_functions.h"
// Member `position`
// Member `velocity`
// Member `acceleration`
// Member `effort`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void state_estimator_msgs__msg__JointStateWithAcceleration__rosidl_typesupport_introspection_c__JointStateWithAcceleration_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  state_estimator_msgs__msg__JointStateWithAcceleration__init(message_memory);
}

void state_estimator_msgs__msg__JointStateWithAcceleration__rosidl_typesupport_introspection_c__JointStateWithAcceleration_fini_function(void * message_memory)
{
  state_estimator_msgs__msg__JointStateWithAcceleration__fini(message_memory);
}

size_t state_estimator_msgs__msg__JointStateWithAcceleration__rosidl_typesupport_introspection_c__size_function__JointStateWithAcceleration__name(
  const void * untyped_member)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return member->size;
}

const void * state_estimator_msgs__msg__JointStateWithAcceleration__rosidl_typesupport_introspection_c__get_const_function__JointStateWithAcceleration__name(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void * state_estimator_msgs__msg__JointStateWithAcceleration__rosidl_typesupport_introspection_c__get_function__JointStateWithAcceleration__name(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void state_estimator_msgs__msg__JointStateWithAcceleration__rosidl_typesupport_introspection_c__fetch_function__JointStateWithAcceleration__name(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const rosidl_runtime_c__String * item =
    ((const rosidl_runtime_c__String *)
    state_estimator_msgs__msg__JointStateWithAcceleration__rosidl_typesupport_introspection_c__get_const_function__JointStateWithAcceleration__name(untyped_member, index));
  rosidl_runtime_c__String * value =
    (rosidl_runtime_c__String *)(untyped_value);
  *value = *item;
}

void state_estimator_msgs__msg__JointStateWithAcceleration__rosidl_typesupport_introspection_c__assign_function__JointStateWithAcceleration__name(
  void * untyped_member, size_t index, const void * untyped_value)
{
  rosidl_runtime_c__String * item =
    ((rosidl_runtime_c__String *)
    state_estimator_msgs__msg__JointStateWithAcceleration__rosidl_typesupport_introspection_c__get_function__JointStateWithAcceleration__name(untyped_member, index));
  const rosidl_runtime_c__String * value =
    (const rosidl_runtime_c__String *)(untyped_value);
  *item = *value;
}

bool state_estimator_msgs__msg__JointStateWithAcceleration__rosidl_typesupport_introspection_c__resize_function__JointStateWithAcceleration__name(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  rosidl_runtime_c__String__Sequence__fini(member);
  return rosidl_runtime_c__String__Sequence__init(member, size);
}

size_t state_estimator_msgs__msg__JointStateWithAcceleration__rosidl_typesupport_introspection_c__size_function__JointStateWithAcceleration__position(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * state_estimator_msgs__msg__JointStateWithAcceleration__rosidl_typesupport_introspection_c__get_const_function__JointStateWithAcceleration__position(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * state_estimator_msgs__msg__JointStateWithAcceleration__rosidl_typesupport_introspection_c__get_function__JointStateWithAcceleration__position(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void state_estimator_msgs__msg__JointStateWithAcceleration__rosidl_typesupport_introspection_c__fetch_function__JointStateWithAcceleration__position(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    state_estimator_msgs__msg__JointStateWithAcceleration__rosidl_typesupport_introspection_c__get_const_function__JointStateWithAcceleration__position(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void state_estimator_msgs__msg__JointStateWithAcceleration__rosidl_typesupport_introspection_c__assign_function__JointStateWithAcceleration__position(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    state_estimator_msgs__msg__JointStateWithAcceleration__rosidl_typesupport_introspection_c__get_function__JointStateWithAcceleration__position(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool state_estimator_msgs__msg__JointStateWithAcceleration__rosidl_typesupport_introspection_c__resize_function__JointStateWithAcceleration__position(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

size_t state_estimator_msgs__msg__JointStateWithAcceleration__rosidl_typesupport_introspection_c__size_function__JointStateWithAcceleration__velocity(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * state_estimator_msgs__msg__JointStateWithAcceleration__rosidl_typesupport_introspection_c__get_const_function__JointStateWithAcceleration__velocity(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * state_estimator_msgs__msg__JointStateWithAcceleration__rosidl_typesupport_introspection_c__get_function__JointStateWithAcceleration__velocity(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void state_estimator_msgs__msg__JointStateWithAcceleration__rosidl_typesupport_introspection_c__fetch_function__JointStateWithAcceleration__velocity(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    state_estimator_msgs__msg__JointStateWithAcceleration__rosidl_typesupport_introspection_c__get_const_function__JointStateWithAcceleration__velocity(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void state_estimator_msgs__msg__JointStateWithAcceleration__rosidl_typesupport_introspection_c__assign_function__JointStateWithAcceleration__velocity(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    state_estimator_msgs__msg__JointStateWithAcceleration__rosidl_typesupport_introspection_c__get_function__JointStateWithAcceleration__velocity(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool state_estimator_msgs__msg__JointStateWithAcceleration__rosidl_typesupport_introspection_c__resize_function__JointStateWithAcceleration__velocity(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

size_t state_estimator_msgs__msg__JointStateWithAcceleration__rosidl_typesupport_introspection_c__size_function__JointStateWithAcceleration__acceleration(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * state_estimator_msgs__msg__JointStateWithAcceleration__rosidl_typesupport_introspection_c__get_const_function__JointStateWithAcceleration__acceleration(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * state_estimator_msgs__msg__JointStateWithAcceleration__rosidl_typesupport_introspection_c__get_function__JointStateWithAcceleration__acceleration(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void state_estimator_msgs__msg__JointStateWithAcceleration__rosidl_typesupport_introspection_c__fetch_function__JointStateWithAcceleration__acceleration(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    state_estimator_msgs__msg__JointStateWithAcceleration__rosidl_typesupport_introspection_c__get_const_function__JointStateWithAcceleration__acceleration(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void state_estimator_msgs__msg__JointStateWithAcceleration__rosidl_typesupport_introspection_c__assign_function__JointStateWithAcceleration__acceleration(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    state_estimator_msgs__msg__JointStateWithAcceleration__rosidl_typesupport_introspection_c__get_function__JointStateWithAcceleration__acceleration(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool state_estimator_msgs__msg__JointStateWithAcceleration__rosidl_typesupport_introspection_c__resize_function__JointStateWithAcceleration__acceleration(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

size_t state_estimator_msgs__msg__JointStateWithAcceleration__rosidl_typesupport_introspection_c__size_function__JointStateWithAcceleration__effort(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * state_estimator_msgs__msg__JointStateWithAcceleration__rosidl_typesupport_introspection_c__get_const_function__JointStateWithAcceleration__effort(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * state_estimator_msgs__msg__JointStateWithAcceleration__rosidl_typesupport_introspection_c__get_function__JointStateWithAcceleration__effort(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void state_estimator_msgs__msg__JointStateWithAcceleration__rosidl_typesupport_introspection_c__fetch_function__JointStateWithAcceleration__effort(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    state_estimator_msgs__msg__JointStateWithAcceleration__rosidl_typesupport_introspection_c__get_const_function__JointStateWithAcceleration__effort(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void state_estimator_msgs__msg__JointStateWithAcceleration__rosidl_typesupport_introspection_c__assign_function__JointStateWithAcceleration__effort(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    state_estimator_msgs__msg__JointStateWithAcceleration__rosidl_typesupport_introspection_c__get_function__JointStateWithAcceleration__effort(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool state_estimator_msgs__msg__JointStateWithAcceleration__rosidl_typesupport_introspection_c__resize_function__JointStateWithAcceleration__effort(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember state_estimator_msgs__msg__JointStateWithAcceleration__rosidl_typesupport_introspection_c__JointStateWithAcceleration_message_member_array[6] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(state_estimator_msgs__msg__JointStateWithAcceleration, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "name",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(state_estimator_msgs__msg__JointStateWithAcceleration, name),  // bytes offset in struct
    NULL,  // default value
    state_estimator_msgs__msg__JointStateWithAcceleration__rosidl_typesupport_introspection_c__size_function__JointStateWithAcceleration__name,  // size() function pointer
    state_estimator_msgs__msg__JointStateWithAcceleration__rosidl_typesupport_introspection_c__get_const_function__JointStateWithAcceleration__name,  // get_const(index) function pointer
    state_estimator_msgs__msg__JointStateWithAcceleration__rosidl_typesupport_introspection_c__get_function__JointStateWithAcceleration__name,  // get(index) function pointer
    state_estimator_msgs__msg__JointStateWithAcceleration__rosidl_typesupport_introspection_c__fetch_function__JointStateWithAcceleration__name,  // fetch(index, &value) function pointer
    state_estimator_msgs__msg__JointStateWithAcceleration__rosidl_typesupport_introspection_c__assign_function__JointStateWithAcceleration__name,  // assign(index, value) function pointer
    state_estimator_msgs__msg__JointStateWithAcceleration__rosidl_typesupport_introspection_c__resize_function__JointStateWithAcceleration__name  // resize(index) function pointer
  },
  {
    "position",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(state_estimator_msgs__msg__JointStateWithAcceleration, position),  // bytes offset in struct
    NULL,  // default value
    state_estimator_msgs__msg__JointStateWithAcceleration__rosidl_typesupport_introspection_c__size_function__JointStateWithAcceleration__position,  // size() function pointer
    state_estimator_msgs__msg__JointStateWithAcceleration__rosidl_typesupport_introspection_c__get_const_function__JointStateWithAcceleration__position,  // get_const(index) function pointer
    state_estimator_msgs__msg__JointStateWithAcceleration__rosidl_typesupport_introspection_c__get_function__JointStateWithAcceleration__position,  // get(index) function pointer
    state_estimator_msgs__msg__JointStateWithAcceleration__rosidl_typesupport_introspection_c__fetch_function__JointStateWithAcceleration__position,  // fetch(index, &value) function pointer
    state_estimator_msgs__msg__JointStateWithAcceleration__rosidl_typesupport_introspection_c__assign_function__JointStateWithAcceleration__position,  // assign(index, value) function pointer
    state_estimator_msgs__msg__JointStateWithAcceleration__rosidl_typesupport_introspection_c__resize_function__JointStateWithAcceleration__position  // resize(index) function pointer
  },
  {
    "velocity",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(state_estimator_msgs__msg__JointStateWithAcceleration, velocity),  // bytes offset in struct
    NULL,  // default value
    state_estimator_msgs__msg__JointStateWithAcceleration__rosidl_typesupport_introspection_c__size_function__JointStateWithAcceleration__velocity,  // size() function pointer
    state_estimator_msgs__msg__JointStateWithAcceleration__rosidl_typesupport_introspection_c__get_const_function__JointStateWithAcceleration__velocity,  // get_const(index) function pointer
    state_estimator_msgs__msg__JointStateWithAcceleration__rosidl_typesupport_introspection_c__get_function__JointStateWithAcceleration__velocity,  // get(index) function pointer
    state_estimator_msgs__msg__JointStateWithAcceleration__rosidl_typesupport_introspection_c__fetch_function__JointStateWithAcceleration__velocity,  // fetch(index, &value) function pointer
    state_estimator_msgs__msg__JointStateWithAcceleration__rosidl_typesupport_introspection_c__assign_function__JointStateWithAcceleration__velocity,  // assign(index, value) function pointer
    state_estimator_msgs__msg__JointStateWithAcceleration__rosidl_typesupport_introspection_c__resize_function__JointStateWithAcceleration__velocity  // resize(index) function pointer
  },
  {
    "acceleration",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(state_estimator_msgs__msg__JointStateWithAcceleration, acceleration),  // bytes offset in struct
    NULL,  // default value
    state_estimator_msgs__msg__JointStateWithAcceleration__rosidl_typesupport_introspection_c__size_function__JointStateWithAcceleration__acceleration,  // size() function pointer
    state_estimator_msgs__msg__JointStateWithAcceleration__rosidl_typesupport_introspection_c__get_const_function__JointStateWithAcceleration__acceleration,  // get_const(index) function pointer
    state_estimator_msgs__msg__JointStateWithAcceleration__rosidl_typesupport_introspection_c__get_function__JointStateWithAcceleration__acceleration,  // get(index) function pointer
    state_estimator_msgs__msg__JointStateWithAcceleration__rosidl_typesupport_introspection_c__fetch_function__JointStateWithAcceleration__acceleration,  // fetch(index, &value) function pointer
    state_estimator_msgs__msg__JointStateWithAcceleration__rosidl_typesupport_introspection_c__assign_function__JointStateWithAcceleration__acceleration,  // assign(index, value) function pointer
    state_estimator_msgs__msg__JointStateWithAcceleration__rosidl_typesupport_introspection_c__resize_function__JointStateWithAcceleration__acceleration  // resize(index) function pointer
  },
  {
    "effort",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(state_estimator_msgs__msg__JointStateWithAcceleration, effort),  // bytes offset in struct
    NULL,  // default value
    state_estimator_msgs__msg__JointStateWithAcceleration__rosidl_typesupport_introspection_c__size_function__JointStateWithAcceleration__effort,  // size() function pointer
    state_estimator_msgs__msg__JointStateWithAcceleration__rosidl_typesupport_introspection_c__get_const_function__JointStateWithAcceleration__effort,  // get_const(index) function pointer
    state_estimator_msgs__msg__JointStateWithAcceleration__rosidl_typesupport_introspection_c__get_function__JointStateWithAcceleration__effort,  // get(index) function pointer
    state_estimator_msgs__msg__JointStateWithAcceleration__rosidl_typesupport_introspection_c__fetch_function__JointStateWithAcceleration__effort,  // fetch(index, &value) function pointer
    state_estimator_msgs__msg__JointStateWithAcceleration__rosidl_typesupport_introspection_c__assign_function__JointStateWithAcceleration__effort,  // assign(index, value) function pointer
    state_estimator_msgs__msg__JointStateWithAcceleration__rosidl_typesupport_introspection_c__resize_function__JointStateWithAcceleration__effort  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers state_estimator_msgs__msg__JointStateWithAcceleration__rosidl_typesupport_introspection_c__JointStateWithAcceleration_message_members = {
  "state_estimator_msgs__msg",  // message namespace
  "JointStateWithAcceleration",  // message name
  6,  // number of fields
  sizeof(state_estimator_msgs__msg__JointStateWithAcceleration),
  false,  // has_any_key_member_
  state_estimator_msgs__msg__JointStateWithAcceleration__rosidl_typesupport_introspection_c__JointStateWithAcceleration_message_member_array,  // message members
  state_estimator_msgs__msg__JointStateWithAcceleration__rosidl_typesupport_introspection_c__JointStateWithAcceleration_init_function,  // function to initialize message memory (memory has to be allocated)
  state_estimator_msgs__msg__JointStateWithAcceleration__rosidl_typesupport_introspection_c__JointStateWithAcceleration_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t state_estimator_msgs__msg__JointStateWithAcceleration__rosidl_typesupport_introspection_c__JointStateWithAcceleration_message_type_support_handle = {
  0,
  &state_estimator_msgs__msg__JointStateWithAcceleration__rosidl_typesupport_introspection_c__JointStateWithAcceleration_message_members,
  get_message_typesupport_handle_function,
  &state_estimator_msgs__msg__JointStateWithAcceleration__get_type_hash,
  &state_estimator_msgs__msg__JointStateWithAcceleration__get_type_description,
  &state_estimator_msgs__msg__JointStateWithAcceleration__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_state_estimator_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, state_estimator_msgs, msg, JointStateWithAcceleration)() {
  state_estimator_msgs__msg__JointStateWithAcceleration__rosidl_typesupport_introspection_c__JointStateWithAcceleration_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  if (!state_estimator_msgs__msg__JointStateWithAcceleration__rosidl_typesupport_introspection_c__JointStateWithAcceleration_message_type_support_handle.typesupport_identifier) {
    state_estimator_msgs__msg__JointStateWithAcceleration__rosidl_typesupport_introspection_c__JointStateWithAcceleration_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &state_estimator_msgs__msg__JointStateWithAcceleration__rosidl_typesupport_introspection_c__JointStateWithAcceleration_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
