// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from state_estimator_msgs:msg/LegOdometry.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "state_estimator_msgs/msg/leg_odometry.h"


#ifndef STATE_ESTIMATOR_MSGS__MSG__DETAIL__LEG_ODOMETRY__STRUCT_H_
#define STATE_ESTIMATOR_MSGS__MSG__DETAIL__LEG_ODOMETRY__STRUCT_H_

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

/// Struct defined in msg/LegOdometry in the package state_estimator_msgs.
typedef struct state_estimator_msgs__msg__LegOdometry
{
  std_msgs__msg__Header header;
  double lin_vel_lf[3];
  double lin_vel_rf[3];
  double lin_vel_lh[3];
  double lin_vel_rh[3];
  double base_velocity[3];
} state_estimator_msgs__msg__LegOdometry;

// Struct for a sequence of state_estimator_msgs__msg__LegOdometry.
typedef struct state_estimator_msgs__msg__LegOdometry__Sequence
{
  state_estimator_msgs__msg__LegOdometry * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} state_estimator_msgs__msg__LegOdometry__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // STATE_ESTIMATOR_MSGS__MSG__DETAIL__LEG_ODOMETRY__STRUCT_H_
