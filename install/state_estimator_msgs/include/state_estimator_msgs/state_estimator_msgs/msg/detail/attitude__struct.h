// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from state_estimator_msgs:msg/Attitude.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "state_estimator_msgs/msg/attitude.h"


#ifndef STATE_ESTIMATOR_MSGS__MSG__DETAIL__ATTITUDE__STRUCT_H_
#define STATE_ESTIMATOR_MSGS__MSG__DETAIL__ATTITUDE__STRUCT_H_

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

/// Struct defined in msg/Attitude in the package state_estimator_msgs.
typedef struct state_estimator_msgs__msg__Attitude
{
  std_msgs__msg__Header header;
  double quaternion[4];
  double roll_deg;
  double pitch_deg;
  double yaw_deg;
  double angular_velocity[3];
} state_estimator_msgs__msg__Attitude;

// Struct for a sequence of state_estimator_msgs__msg__Attitude.
typedef struct state_estimator_msgs__msg__Attitude__Sequence
{
  state_estimator_msgs__msg__Attitude * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} state_estimator_msgs__msg__Attitude__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // STATE_ESTIMATOR_MSGS__MSG__DETAIL__ATTITUDE__STRUCT_H_
