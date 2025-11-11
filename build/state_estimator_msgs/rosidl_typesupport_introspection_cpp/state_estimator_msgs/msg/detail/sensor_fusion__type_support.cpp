// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from state_estimator_msgs:msg/SensorFusion.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "state_estimator_msgs/msg/detail/sensor_fusion__functions.h"
#include "state_estimator_msgs/msg/detail/sensor_fusion__struct.hpp"
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

void SensorFusion_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) state_estimator_msgs::msg::SensorFusion(_init);
}

void SensorFusion_fini_function(void * message_memory)
{
  auto typed_message = static_cast<state_estimator_msgs::msg::SensorFusion *>(message_memory);
  typed_message->~SensorFusion();
}

size_t size_function__SensorFusion__position(const void * untyped_member)
{
  (void)untyped_member;
  return 3;
}

const void * get_const_function__SensorFusion__position(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<double, 3> *>(untyped_member);
  return &member[index];
}

void * get_function__SensorFusion__position(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<double, 3> *>(untyped_member);
  return &member[index];
}

void fetch_function__SensorFusion__position(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__SensorFusion__position(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__SensorFusion__position(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__SensorFusion__position(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

size_t size_function__SensorFusion__linear_velocity(const void * untyped_member)
{
  (void)untyped_member;
  return 3;
}

const void * get_const_function__SensorFusion__linear_velocity(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<double, 3> *>(untyped_member);
  return &member[index];
}

void * get_function__SensorFusion__linear_velocity(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<double, 3> *>(untyped_member);
  return &member[index];
}

void fetch_function__SensorFusion__linear_velocity(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__SensorFusion__linear_velocity(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__SensorFusion__linear_velocity(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__SensorFusion__linear_velocity(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember SensorFusion_message_member_array[3] = {
  {
    "header",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<std_msgs::msg::Header>(),  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(state_estimator_msgs::msg::SensorFusion, header),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "position",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    true,  // is array
    3,  // array size
    false,  // is upper bound
    offsetof(state_estimator_msgs::msg::SensorFusion, position),  // bytes offset in struct
    nullptr,  // default value
    size_function__SensorFusion__position,  // size() function pointer
    get_const_function__SensorFusion__position,  // get_const(index) function pointer
    get_function__SensorFusion__position,  // get(index) function pointer
    fetch_function__SensorFusion__position,  // fetch(index, &value) function pointer
    assign_function__SensorFusion__position,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "linear_velocity",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    true,  // is array
    3,  // array size
    false,  // is upper bound
    offsetof(state_estimator_msgs::msg::SensorFusion, linear_velocity),  // bytes offset in struct
    nullptr,  // default value
    size_function__SensorFusion__linear_velocity,  // size() function pointer
    get_const_function__SensorFusion__linear_velocity,  // get_const(index) function pointer
    get_function__SensorFusion__linear_velocity,  // get(index) function pointer
    fetch_function__SensorFusion__linear_velocity,  // fetch(index, &value) function pointer
    assign_function__SensorFusion__linear_velocity,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers SensorFusion_message_members = {
  "state_estimator_msgs::msg",  // message namespace
  "SensorFusion",  // message name
  3,  // number of fields
  sizeof(state_estimator_msgs::msg::SensorFusion),
  false,  // has_any_key_member_
  SensorFusion_message_member_array,  // message members
  SensorFusion_init_function,  // function to initialize message memory (memory has to be allocated)
  SensorFusion_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t SensorFusion_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &SensorFusion_message_members,
  get_message_typesupport_handle_function,
  &state_estimator_msgs__msg__SensorFusion__get_type_hash,
  &state_estimator_msgs__msg__SensorFusion__get_type_description,
  &state_estimator_msgs__msg__SensorFusion__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace state_estimator_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<state_estimator_msgs::msg::SensorFusion>()
{
  return &::state_estimator_msgs::msg::rosidl_typesupport_introspection_cpp::SensorFusion_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, state_estimator_msgs, msg, SensorFusion)() {
  return &::state_estimator_msgs::msg::rosidl_typesupport_introspection_cpp::SensorFusion_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
