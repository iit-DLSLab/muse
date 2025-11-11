// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from state_estimator_msgs:msg/JointStateWithAcceleration.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "state_estimator_msgs/msg/joint_state_with_acceleration.h"


#ifndef STATE_ESTIMATOR_MSGS__MSG__DETAIL__JOINT_STATE_WITH_ACCELERATION__STRUCT_H_
#define STATE_ESTIMATOR_MSGS__MSG__DETAIL__JOINT_STATE_WITH_ACCELERATION__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'name'
#include "rosidl_runtime_c/string.h"
// Member 'position'
// Member 'velocity'
// Member 'acceleration'
// Member 'effort'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in msg/JointStateWithAcceleration in the package state_estimator_msgs.
typedef struct state_estimator_msgs__msg__JointStateWithAcceleration
{
  std_msgs__msg__Header header;
  rosidl_runtime_c__String__Sequence name;
  rosidl_runtime_c__double__Sequence position;
  rosidl_runtime_c__double__Sequence velocity;
  rosidl_runtime_c__double__Sequence acceleration;
  rosidl_runtime_c__double__Sequence effort;
} state_estimator_msgs__msg__JointStateWithAcceleration;

// Struct for a sequence of state_estimator_msgs__msg__JointStateWithAcceleration.
typedef struct state_estimator_msgs__msg__JointStateWithAcceleration__Sequence
{
  state_estimator_msgs__msg__JointStateWithAcceleration * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} state_estimator_msgs__msg__JointStateWithAcceleration__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // STATE_ESTIMATOR_MSGS__MSG__DETAIL__JOINT_STATE_WITH_ACCELERATION__STRUCT_H_
