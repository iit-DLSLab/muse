// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from state_estimator_msgs:msg/Attitude.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "state_estimator_msgs/msg/attitude.hpp"


#ifndef STATE_ESTIMATOR_MSGS__MSG__DETAIL__ATTITUDE__BUILDER_HPP_
#define STATE_ESTIMATOR_MSGS__MSG__DETAIL__ATTITUDE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "state_estimator_msgs/msg/detail/attitude__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace state_estimator_msgs
{

namespace msg
{

namespace builder
{

class Init_Attitude_angular_velocity
{
public:
  explicit Init_Attitude_angular_velocity(::state_estimator_msgs::msg::Attitude & msg)
  : msg_(msg)
  {}
  ::state_estimator_msgs::msg::Attitude angular_velocity(::state_estimator_msgs::msg::Attitude::_angular_velocity_type arg)
  {
    msg_.angular_velocity = std::move(arg);
    return std::move(msg_);
  }

private:
  ::state_estimator_msgs::msg::Attitude msg_;
};

class Init_Attitude_yaw_deg
{
public:
  explicit Init_Attitude_yaw_deg(::state_estimator_msgs::msg::Attitude & msg)
  : msg_(msg)
  {}
  Init_Attitude_angular_velocity yaw_deg(::state_estimator_msgs::msg::Attitude::_yaw_deg_type arg)
  {
    msg_.yaw_deg = std::move(arg);
    return Init_Attitude_angular_velocity(msg_);
  }

private:
  ::state_estimator_msgs::msg::Attitude msg_;
};

class Init_Attitude_pitch_deg
{
public:
  explicit Init_Attitude_pitch_deg(::state_estimator_msgs::msg::Attitude & msg)
  : msg_(msg)
  {}
  Init_Attitude_yaw_deg pitch_deg(::state_estimator_msgs::msg::Attitude::_pitch_deg_type arg)
  {
    msg_.pitch_deg = std::move(arg);
    return Init_Attitude_yaw_deg(msg_);
  }

private:
  ::state_estimator_msgs::msg::Attitude msg_;
};

class Init_Attitude_roll_deg
{
public:
  explicit Init_Attitude_roll_deg(::state_estimator_msgs::msg::Attitude & msg)
  : msg_(msg)
  {}
  Init_Attitude_pitch_deg roll_deg(::state_estimator_msgs::msg::Attitude::_roll_deg_type arg)
  {
    msg_.roll_deg = std::move(arg);
    return Init_Attitude_pitch_deg(msg_);
  }

private:
  ::state_estimator_msgs::msg::Attitude msg_;
};

class Init_Attitude_quaternion
{
public:
  explicit Init_Attitude_quaternion(::state_estimator_msgs::msg::Attitude & msg)
  : msg_(msg)
  {}
  Init_Attitude_roll_deg quaternion(::state_estimator_msgs::msg::Attitude::_quaternion_type arg)
  {
    msg_.quaternion = std::move(arg);
    return Init_Attitude_roll_deg(msg_);
  }

private:
  ::state_estimator_msgs::msg::Attitude msg_;
};

class Init_Attitude_header
{
public:
  Init_Attitude_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Attitude_quaternion header(::state_estimator_msgs::msg::Attitude::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_Attitude_quaternion(msg_);
  }

private:
  ::state_estimator_msgs::msg::Attitude msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::state_estimator_msgs::msg::Attitude>()
{
  return state_estimator_msgs::msg::builder::Init_Attitude_header();
}

}  // namespace state_estimator_msgs

#endif  // STATE_ESTIMATOR_MSGS__MSG__DETAIL__ATTITUDE__BUILDER_HPP_
