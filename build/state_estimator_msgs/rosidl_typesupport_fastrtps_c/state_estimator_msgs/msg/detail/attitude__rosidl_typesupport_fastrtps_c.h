// generated from rosidl_typesupport_fastrtps_c/resource/idl__rosidl_typesupport_fastrtps_c.h.em
// with input from state_estimator_msgs:msg/Attitude.idl
// generated code does not contain a copyright notice
#ifndef STATE_ESTIMATOR_MSGS__MSG__DETAIL__ATTITUDE__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_
#define STATE_ESTIMATOR_MSGS__MSG__DETAIL__ATTITUDE__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_


#include <stddef.h>
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "state_estimator_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "state_estimator_msgs/msg/detail/attitude__struct.h"
#include "fastcdr/Cdr.h"

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_state_estimator_msgs
bool cdr_serialize_state_estimator_msgs__msg__Attitude(
  const state_estimator_msgs__msg__Attitude * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_state_estimator_msgs
bool cdr_deserialize_state_estimator_msgs__msg__Attitude(
  eprosima::fastcdr::Cdr &,
  state_estimator_msgs__msg__Attitude * ros_message);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_state_estimator_msgs
size_t get_serialized_size_state_estimator_msgs__msg__Attitude(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_state_estimator_msgs
size_t max_serialized_size_state_estimator_msgs__msg__Attitude(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_state_estimator_msgs
bool cdr_serialize_key_state_estimator_msgs__msg__Attitude(
  const state_estimator_msgs__msg__Attitude * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_state_estimator_msgs
size_t get_serialized_size_key_state_estimator_msgs__msg__Attitude(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_state_estimator_msgs
size_t max_serialized_size_key_state_estimator_msgs__msg__Attitude(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_state_estimator_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, state_estimator_msgs, msg, Attitude)();

#ifdef __cplusplus
}
#endif

#endif  // STATE_ESTIMATOR_MSGS__MSG__DETAIL__ATTITUDE__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_
