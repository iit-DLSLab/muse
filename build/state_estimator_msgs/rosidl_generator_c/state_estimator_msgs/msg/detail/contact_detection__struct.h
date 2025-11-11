// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from state_estimator_msgs:msg/ContactDetection.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "state_estimator_msgs/msg/contact_detection.h"


#ifndef STATE_ESTIMATOR_MSGS__MSG__DETAIL__CONTACT_DETECTION__STRUCT_H_
#define STATE_ESTIMATOR_MSGS__MSG__DETAIL__CONTACT_DETECTION__STRUCT_H_

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

/// Struct defined in msg/ContactDetection in the package state_estimator_msgs.
typedef struct state_estimator_msgs__msg__ContactDetection
{
  std_msgs__msg__Header header;
  bool stance_lf;
  bool stance_rf;
  bool stance_lh;
  bool stance_rh;
} state_estimator_msgs__msg__ContactDetection;

// Struct for a sequence of state_estimator_msgs__msg__ContactDetection.
typedef struct state_estimator_msgs__msg__ContactDetection__Sequence
{
  state_estimator_msgs__msg__ContactDetection * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} state_estimator_msgs__msg__ContactDetection__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // STATE_ESTIMATOR_MSGS__MSG__DETAIL__CONTACT_DETECTION__STRUCT_H_
