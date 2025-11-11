// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from state_estimator_msgs:msg/JointStateWithAcceleration.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "state_estimator_msgs/msg/detail/joint_state_with_acceleration__functions.h"
#include "state_estimator_msgs/msg/detail/joint_state_with_acceleration__struct.hpp"
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

void JointStateWithAcceleration_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) state_estimator_msgs::msg::JointStateWithAcceleration(_init);
}

void JointStateWithAcceleration_fini_function(void * message_memory)
{
  auto typed_message = static_cast<state_estimator_msgs::msg::JointStateWithAcceleration *>(message_memory);
  typed_message->~JointStateWithAcceleration();
}

size_t size_function__JointStateWithAcceleration__name(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<std::string> *>(untyped_member);
  return member->size();
}

const void * get_const_function__JointStateWithAcceleration__name(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<std::string> *>(untyped_member);
  return &member[index];
}

void * get_function__JointStateWithAcceleration__name(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<std::string> *>(untyped_member);
  return &member[index];
}

void fetch_function__JointStateWithAcceleration__name(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const std::string *>(
    get_const_function__JointStateWithAcceleration__name(untyped_member, index));
  auto & value = *reinterpret_cast<std::string *>(untyped_value);
  value = item;
}

void assign_function__JointStateWithAcceleration__name(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<std::string *>(
    get_function__JointStateWithAcceleration__name(untyped_member, index));
  const auto & value = *reinterpret_cast<const std::string *>(untyped_value);
  item = value;
}

void resize_function__JointStateWithAcceleration__name(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<std::string> *>(untyped_member);
  member->resize(size);
}

size_t size_function__JointStateWithAcceleration__position(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<double> *>(untyped_member);
  return member->size();
}

const void * get_const_function__JointStateWithAcceleration__position(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<double> *>(untyped_member);
  return &member[index];
}

void * get_function__JointStateWithAcceleration__position(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<double> *>(untyped_member);
  return &member[index];
}

void fetch_function__JointStateWithAcceleration__position(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__JointStateWithAcceleration__position(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__JointStateWithAcceleration__position(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__JointStateWithAcceleration__position(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

void resize_function__JointStateWithAcceleration__position(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<double> *>(untyped_member);
  member->resize(size);
}

size_t size_function__JointStateWithAcceleration__velocity(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<double> *>(untyped_member);
  return member->size();
}

const void * get_const_function__JointStateWithAcceleration__velocity(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<double> *>(untyped_member);
  return &member[index];
}

void * get_function__JointStateWithAcceleration__velocity(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<double> *>(untyped_member);
  return &member[index];
}

void fetch_function__JointStateWithAcceleration__velocity(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__JointStateWithAcceleration__velocity(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__JointStateWithAcceleration__velocity(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__JointStateWithAcceleration__velocity(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

void resize_function__JointStateWithAcceleration__velocity(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<double> *>(untyped_member);
  member->resize(size);
}

size_t size_function__JointStateWithAcceleration__acceleration(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<double> *>(untyped_member);
  return member->size();
}

const void * get_const_function__JointStateWithAcceleration__acceleration(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<double> *>(untyped_member);
  return &member[index];
}

void * get_function__JointStateWithAcceleration__acceleration(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<double> *>(untyped_member);
  return &member[index];
}

void fetch_function__JointStateWithAcceleration__acceleration(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__JointStateWithAcceleration__acceleration(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__JointStateWithAcceleration__acceleration(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__JointStateWithAcceleration__acceleration(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

void resize_function__JointStateWithAcceleration__acceleration(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<double> *>(untyped_member);
  member->resize(size);
}

size_t size_function__JointStateWithAcceleration__effort(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<double> *>(untyped_member);
  return member->size();
}

const void * get_const_function__JointStateWithAcceleration__effort(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<double> *>(untyped_member);
  return &member[index];
}

void * get_function__JointStateWithAcceleration__effort(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<double> *>(untyped_member);
  return &member[index];
}

void fetch_function__JointStateWithAcceleration__effort(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__JointStateWithAcceleration__effort(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__JointStateWithAcceleration__effort(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__JointStateWithAcceleration__effort(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

void resize_function__JointStateWithAcceleration__effort(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<double> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember JointStateWithAcceleration_message_member_array[6] = {
  {
    "header",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<std_msgs::msg::Header>(),  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(state_estimator_msgs::msg::JointStateWithAcceleration, header),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "name",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(state_estimator_msgs::msg::JointStateWithAcceleration, name),  // bytes offset in struct
    nullptr,  // default value
    size_function__JointStateWithAcceleration__name,  // size() function pointer
    get_const_function__JointStateWithAcceleration__name,  // get_const(index) function pointer
    get_function__JointStateWithAcceleration__name,  // get(index) function pointer
    fetch_function__JointStateWithAcceleration__name,  // fetch(index, &value) function pointer
    assign_function__JointStateWithAcceleration__name,  // assign(index, value) function pointer
    resize_function__JointStateWithAcceleration__name  // resize(index) function pointer
  },
  {
    "position",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(state_estimator_msgs::msg::JointStateWithAcceleration, position),  // bytes offset in struct
    nullptr,  // default value
    size_function__JointStateWithAcceleration__position,  // size() function pointer
    get_const_function__JointStateWithAcceleration__position,  // get_const(index) function pointer
    get_function__JointStateWithAcceleration__position,  // get(index) function pointer
    fetch_function__JointStateWithAcceleration__position,  // fetch(index, &value) function pointer
    assign_function__JointStateWithAcceleration__position,  // assign(index, value) function pointer
    resize_function__JointStateWithAcceleration__position  // resize(index) function pointer
  },
  {
    "velocity",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(state_estimator_msgs::msg::JointStateWithAcceleration, velocity),  // bytes offset in struct
    nullptr,  // default value
    size_function__JointStateWithAcceleration__velocity,  // size() function pointer
    get_const_function__JointStateWithAcceleration__velocity,  // get_const(index) function pointer
    get_function__JointStateWithAcceleration__velocity,  // get(index) function pointer
    fetch_function__JointStateWithAcceleration__velocity,  // fetch(index, &value) function pointer
    assign_function__JointStateWithAcceleration__velocity,  // assign(index, value) function pointer
    resize_function__JointStateWithAcceleration__velocity  // resize(index) function pointer
  },
  {
    "acceleration",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(state_estimator_msgs::msg::JointStateWithAcceleration, acceleration),  // bytes offset in struct
    nullptr,  // default value
    size_function__JointStateWithAcceleration__acceleration,  // size() function pointer
    get_const_function__JointStateWithAcceleration__acceleration,  // get_const(index) function pointer
    get_function__JointStateWithAcceleration__acceleration,  // get(index) function pointer
    fetch_function__JointStateWithAcceleration__acceleration,  // fetch(index, &value) function pointer
    assign_function__JointStateWithAcceleration__acceleration,  // assign(index, value) function pointer
    resize_function__JointStateWithAcceleration__acceleration  // resize(index) function pointer
  },
  {
    "effort",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(state_estimator_msgs::msg::JointStateWithAcceleration, effort),  // bytes offset in struct
    nullptr,  // default value
    size_function__JointStateWithAcceleration__effort,  // size() function pointer
    get_const_function__JointStateWithAcceleration__effort,  // get_const(index) function pointer
    get_function__JointStateWithAcceleration__effort,  // get(index) function pointer
    fetch_function__JointStateWithAcceleration__effort,  // fetch(index, &value) function pointer
    assign_function__JointStateWithAcceleration__effort,  // assign(index, value) function pointer
    resize_function__JointStateWithAcceleration__effort  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers JointStateWithAcceleration_message_members = {
  "state_estimator_msgs::msg",  // message namespace
  "JointStateWithAcceleration",  // message name
  6,  // number of fields
  sizeof(state_estimator_msgs::msg::JointStateWithAcceleration),
  false,  // has_any_key_member_
  JointStateWithAcceleration_message_member_array,  // message members
  JointStateWithAcceleration_init_function,  // function to initialize message memory (memory has to be allocated)
  JointStateWithAcceleration_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t JointStateWithAcceleration_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &JointStateWithAcceleration_message_members,
  get_message_typesupport_handle_function,
  &state_estimator_msgs__msg__JointStateWithAcceleration__get_type_hash,
  &state_estimator_msgs__msg__JointStateWithAcceleration__get_type_description,
  &state_estimator_msgs__msg__JointStateWithAcceleration__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace state_estimator_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<state_estimator_msgs::msg::JointStateWithAcceleration>()
{
  return &::state_estimator_msgs::msg::rosidl_typesupport_introspection_cpp::JointStateWithAcceleration_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, state_estimator_msgs, msg, JointStateWithAcceleration)() {
  return &::state_estimator_msgs::msg::rosidl_typesupport_introspection_cpp::JointStateWithAcceleration_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
