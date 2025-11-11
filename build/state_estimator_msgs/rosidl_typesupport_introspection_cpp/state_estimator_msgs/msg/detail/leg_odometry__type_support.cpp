// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from state_estimator_msgs:msg/LegOdometry.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "state_estimator_msgs/msg/detail/leg_odometry__functions.h"
#include "state_estimator_msgs/msg/detail/leg_odometry__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace state_estimator_msgs
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void LegOdometry_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) state_estimator_msgs::msg::LegOdometry(_init);
}

void LegOdometry_fini_function(void * message_memory)
{
  auto typed_message = static_cast<state_estimator_msgs::msg::LegOdometry *>(message_memory);
  typed_message->~LegOdometry();
}

size_t size_function__LegOdometry__lin_vel_lf(const void * untyped_member)
{
  (void)untyped_member;
  return 3;
}

const void * get_const_function__LegOdometry__lin_vel_lf(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<double, 3> *>(untyped_member);
  return &member[index];
}

void * get_function__LegOdometry__lin_vel_lf(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<double, 3> *>(untyped_member);
  return &member[index];
}

void fetch_function__LegOdometry__lin_vel_lf(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__LegOdometry__lin_vel_lf(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__LegOdometry__lin_vel_lf(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__LegOdometry__lin_vel_lf(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

size_t size_function__LegOdometry__lin_vel_rf(const void * untyped_member)
{
  (void)untyped_member;
  return 3;
}

const void * get_const_function__LegOdometry__lin_vel_rf(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<double, 3> *>(untyped_member);
  return &member[index];
}

void * get_function__LegOdometry__lin_vel_rf(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<double, 3> *>(untyped_member);
  return &member[index];
}

void fetch_function__LegOdometry__lin_vel_rf(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__LegOdometry__lin_vel_rf(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__LegOdometry__lin_vel_rf(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__LegOdometry__lin_vel_rf(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

size_t size_function__LegOdometry__lin_vel_lh(const void * untyped_member)
{
  (void)untyped_member;
  return 3;
}

const void * get_const_function__LegOdometry__lin_vel_lh(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<double, 3> *>(untyped_member);
  return &member[index];
}

void * get_function__LegOdometry__lin_vel_lh(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<double, 3> *>(untyped_member);
  return &member[index];
}

void fetch_function__LegOdometry__lin_vel_lh(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__LegOdometry__lin_vel_lh(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__LegOdometry__lin_vel_lh(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__LegOdometry__lin_vel_lh(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

size_t size_function__LegOdometry__lin_vel_rh(const void * untyped_member)
{
  (void)untyped_member;
  return 3;
}

const void * get_const_function__LegOdometry__lin_vel_rh(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<double, 3> *>(untyped_member);
  return &member[index];
}

void * get_function__LegOdometry__lin_vel_rh(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<double, 3> *>(untyped_member);
  return &member[index];
}

void fetch_function__LegOdometry__lin_vel_rh(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__LegOdometry__lin_vel_rh(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__LegOdometry__lin_vel_rh(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__LegOdometry__lin_vel_rh(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

size_t size_function__LegOdometry__base_velocity(const void * untyped_member)
{
  (void)untyped_member;
  return 3;
}

const void * get_const_function__LegOdometry__base_velocity(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<double, 3> *>(untyped_member);
  return &member[index];
}

void * get_function__LegOdometry__base_velocity(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<double, 3> *>(untyped_member);
  return &member[index];
}

void fetch_function__LegOdometry__base_velocity(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__LegOdometry__base_velocity(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__LegOdometry__base_velocity(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__LegOdometry__base_velocity(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember LegOdometry_message_member_array[6] = {
  {
    "header",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<std_msgs::msg::Header>(),  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(state_estimator_msgs::msg::LegOdometry, header),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "lin_vel_lf",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    true,  // is array
    3,  // array size
    false,  // is upper bound
    offsetof(state_estimator_msgs::msg::LegOdometry, lin_vel_lf),  // bytes offset in struct
    nullptr,  // default value
    size_function__LegOdometry__lin_vel_lf,  // size() function pointer
    get_const_function__LegOdometry__lin_vel_lf,  // get_const(index) function pointer
    get_function__LegOdometry__lin_vel_lf,  // get(index) function pointer
    fetch_function__LegOdometry__lin_vel_lf,  // fetch(index, &value) function pointer
    assign_function__LegOdometry__lin_vel_lf,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "lin_vel_rf",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    true,  // is array
    3,  // array size
    false,  // is upper bound
    offsetof(state_estimator_msgs::msg::LegOdometry, lin_vel_rf),  // bytes offset in struct
    nullptr,  // default value
    size_function__LegOdometry__lin_vel_rf,  // size() function pointer
    get_const_function__LegOdometry__lin_vel_rf,  // get_const(index) function pointer
    get_function__LegOdometry__lin_vel_rf,  // get(index) function pointer
    fetch_function__LegOdometry__lin_vel_rf,  // fetch(index, &value) function pointer
    assign_function__LegOdometry__lin_vel_rf,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "lin_vel_lh",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    true,  // is array
    3,  // array size
    false,  // is upper bound
    offsetof(state_estimator_msgs::msg::LegOdometry, lin_vel_lh),  // bytes offset in struct
    nullptr,  // default value
    size_function__LegOdometry__lin_vel_lh,  // size() function pointer
    get_const_function__LegOdometry__lin_vel_lh,  // get_const(index) function pointer
    get_function__LegOdometry__lin_vel_lh,  // get(index) function pointer
    fetch_function__LegOdometry__lin_vel_lh,  // fetch(index, &value) function pointer
    assign_function__LegOdometry__lin_vel_lh,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "lin_vel_rh",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    true,  // is array
    3,  // array size
    false,  // is upper bound
    offsetof(state_estimator_msgs::msg::LegOdometry, lin_vel_rh),  // bytes offset in struct
    nullptr,  // default value
    size_function__LegOdometry__lin_vel_rh,  // size() function pointer
    get_const_function__LegOdometry__lin_vel_rh,  // get_const(index) function pointer
    get_function__LegOdometry__lin_vel_rh,  // get(index) function pointer
    fetch_function__LegOdometry__lin_vel_rh,  // fetch(index, &value) function pointer
    assign_function__LegOdometry__lin_vel_rh,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "base_velocity",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    true,  // is array
    3,  // array size
    false,  // is upper bound
    offsetof(state_estimator_msgs::msg::LegOdometry, base_velocity),  // bytes offset in struct
    nullptr,  // default value
    size_function__LegOdometry__base_velocity,  // size() function pointer
    get_const_function__LegOdometry__base_velocity,  // get_const(index) function pointer
    get_function__LegOdometry__base_velocity,  // get(index) function pointer
    fetch_function__LegOdometry__base_velocity,  // fetch(index, &value) function pointer
    assign_function__LegOdometry__base_velocity,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers LegOdometry_message_members = {
  "state_estimator_msgs::msg",  // message namespace
  "LegOdometry",  // message name
  6,  // number of fields
  sizeof(state_estimator_msgs::msg::LegOdometry),
  false,  // has_any_key_member_
  LegOdometry_message_member_array,  // message members
  LegOdometry_init_function,  // function to initialize message memory (memory has to be allocated)
  LegOdometry_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t LegOdometry_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &LegOdometry_message_members,
  get_message_typesupport_handle_function,
  &state_estimator_msgs__msg__LegOdometry__get_type_hash,
  &state_estimator_msgs__msg__LegOdometry__get_type_description,
  &state_estimator_msgs__msg__LegOdometry__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace state_estimator_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<state_estimator_msgs::msg::LegOdometry>()
{
  return &::state_estimator_msgs::msg::rosidl_typesupport_introspection_cpp::LegOdometry_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, state_estimator_msgs, msg, LegOdometry)() {
  return &::state_estimator_msgs::msg::rosidl_typesupport_introspection_cpp::LegOdometry_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
