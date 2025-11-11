// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from state_estimator_msgs:msg/JointStateWithAcceleration.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "state_estimator_msgs/msg/joint_state_with_acceleration.hpp"


#ifndef STATE_ESTIMATOR_MSGS__MSG__DETAIL__JOINT_STATE_WITH_ACCELERATION__BUILDER_HPP_
#define STATE_ESTIMATOR_MSGS__MSG__DETAIL__JOINT_STATE_WITH_ACCELERATION__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "state_estimator_msgs/msg/detail/joint_state_with_acceleration__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace state_estimator_msgs
{

namespace msg
{

namespace builder
{

class Init_JointStateWithAcceleration_effort
{
public:
  explicit Init_JointStateWithAcceleration_effort(::state_estimator_msgs::msg::JointStateWithAcceleration & msg)
  : msg_(msg)
  {}
  ::state_estimator_msgs::msg::JointStateWithAcceleration effort(::state_estimator_msgs::msg::JointStateWithAcceleration::_effort_type arg)
  {
    msg_.effort = std::move(arg);
    return std::move(msg_);
  }

private:
  ::state_estimator_msgs::msg::JointStateWithAcceleration msg_;
};

class Init_JointStateWithAcceleration_acceleration
{
public:
  explicit Init_JointStateWithAcceleration_acceleration(::state_estimator_msgs::msg::JointStateWithAcceleration & msg)
  : msg_(msg)
  {}
  Init_JointStateWithAcceleration_effort acceleration(::state_estimator_msgs::msg::JointStateWithAcceleration::_acceleration_type arg)
  {
    msg_.acceleration = std::move(arg);
    return Init_JointStateWithAcceleration_effort(msg_);
  }

private:
  ::state_estimator_msgs::msg::JointStateWithAcceleration msg_;
};

class Init_JointStateWithAcceleration_velocity
{
public:
  explicit Init_JointStateWithAcceleration_velocity(::state_estimator_msgs::msg::JointStateWithAcceleration & msg)
  : msg_(msg)
  {}
  Init_JointStateWithAcceleration_acceleration velocity(::state_estimator_msgs::msg::JointStateWithAcceleration::_velocity_type arg)
  {
    msg_.velocity = std::move(arg);
    return Init_JointStateWithAcceleration_acceleration(msg_);
  }

private:
  ::state_estimator_msgs::msg::JointStateWithAcceleration msg_;
};

class Init_JointStateWithAcceleration_position
{
public:
  explicit Init_JointStateWithAcceleration_position(::state_estimator_msgs::msg::JointStateWithAcceleration & msg)
  : msg_(msg)
  {}
  Init_JointStateWithAcceleration_velocity position(::state_estimator_msgs::msg::JointStateWithAcceleration::_position_type arg)
  {
    msg_.position = std::move(arg);
    return Init_JointStateWithAcceleration_velocity(msg_);
  }

private:
  ::state_estimator_msgs::msg::JointStateWithAcceleration msg_;
};

class Init_JointStateWithAcceleration_name
{
public:
  explicit Init_JointStateWithAcceleration_name(::state_estimator_msgs::msg::JointStateWithAcceleration & msg)
  : msg_(msg)
  {}
  Init_JointStateWithAcceleration_position name(::state_estimator_msgs::msg::JointStateWithAcceleration::_name_type arg)
  {
    msg_.name = std::move(arg);
    return Init_JointStateWithAcceleration_position(msg_);
  }

private:
  ::state_estimator_msgs::msg::JointStateWithAcceleration msg_;
};

class Init_JointStateWithAcceleration_header
{
public:
  Init_JointStateWithAcceleration_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_JointStateWithAcceleration_name header(::state_estimator_msgs::msg::JointStateWithAcceleration::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_JointStateWithAcceleration_name(msg_);
  }

private:
  ::state_estimator_msgs::msg::JointStateWithAcceleration msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::state_estimator_msgs::msg::JointStateWithAcceleration>()
{
  return state_estimator_msgs::msg::builder::Init_JointStateWithAcceleration_header();
}

}  // namespace state_estimator_msgs

#endif  // STATE_ESTIMATOR_MSGS__MSG__DETAIL__JOINT_STATE_WITH_ACCELERATION__BUILDER_HPP_
