// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from state_estimator_msgs:msg/SensorFusion.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "state_estimator_msgs/msg/sensor_fusion.h"


#ifndef STATE_ESTIMATOR_MSGS__MSG__DETAIL__SENSOR_FUSION__STRUCT_H_
#define STATE_ESTIMATOR_MSGS__MSG__DETAIL__SENSOR_FUSION__STRUCT_H_

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

/// Struct defined in msg/SensorFusion in the package state_estimator_msgs.
typedef struct state_estimator_msgs__msg__SensorFusion
{
  std_msgs__msg__Header header;
  double position[3];
  double linear_velocity[3];
} state_estimator_msgs__msg__SensorFusion;

// Struct for a sequence of state_estimator_msgs__msg__SensorFusion.
typedef struct state_estimator_msgs__msg__SensorFusion__Sequence
{
  state_estimator_msgs__msg__SensorFusion * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} state_estimator_msgs__msg__SensorFusion__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // STATE_ESTIMATOR_MSGS__MSG__DETAIL__SENSOR_FUSION__STRUCT_H_
