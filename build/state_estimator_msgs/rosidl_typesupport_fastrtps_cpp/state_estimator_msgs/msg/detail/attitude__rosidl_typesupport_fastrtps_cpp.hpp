// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__rosidl_typesupport_fastrtps_cpp.hpp.em
// with input from state_estimator_msgs:msg/Attitude.idl
// generated code does not contain a copyright notice

#ifndef STATE_ESTIMATOR_MSGS__MSG__DETAIL__ATTITUDE__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
#define STATE_ESTIMATOR_MSGS__MSG__DETAIL__ATTITUDE__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

#include <cstddef>
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "state_estimator_msgs/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
#include "state_estimator_msgs/msg/detail/attitude__struct.hpp"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

#include "fastcdr/Cdr.h"

namespace state_estimator_msgs
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_state_estimator_msgs
cdr_serialize(
  const state_estimator_msgs::msg::Attitude & ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_state_estimator_msgs
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  state_estimator_msgs::msg::Attitude & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_state_estimator_msgs
get_serialized_size(
  const state_estimator_msgs::msg::Attitude & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_state_estimator_msgs
max_serialized_size_Attitude(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_state_estimator_msgs
cdr_serialize_key(
  const state_estimator_msgs::msg::Attitude & ros_message,
  eprosima::fastcdr::Cdr &);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_state_estimator_msgs
get_serialized_size_key(
  const state_estimator_msgs::msg::Attitude & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_state_estimator_msgs
max_serialized_size_key_Attitude(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace state_estimator_msgs

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_state_estimator_msgs
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, state_estimator_msgs, msg, Attitude)();

#ifdef __cplusplus
}
#endif

#endif  // STATE_ESTIMATOR_MSGS__MSG__DETAIL__ATTITUDE__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
