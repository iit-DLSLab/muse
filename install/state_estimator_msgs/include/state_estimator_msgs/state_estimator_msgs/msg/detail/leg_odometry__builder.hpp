// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from state_estimator_msgs:msg/LegOdometry.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "state_estimator_msgs/msg/leg_odometry.hpp"


#ifndef STATE_ESTIMATOR_MSGS__MSG__DETAIL__LEG_ODOMETRY__BUILDER_HPP_
#define STATE_ESTIMATOR_MSGS__MSG__DETAIL__LEG_ODOMETRY__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "state_estimator_msgs/msg/detail/leg_odometry__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace state_estimator_msgs
{

namespace msg
{

namespace builder
{

class Init_LegOdometry_base_velocity
{
public:
  explicit Init_LegOdometry_base_velocity(::state_estimator_msgs::msg::LegOdometry & msg)
  : msg_(msg)
  {}
  ::state_estimator_msgs::msg::LegOdometry base_velocity(::state_estimator_msgs::msg::LegOdometry::_base_velocity_type arg)
  {
    msg_.base_velocity = std::move(arg);
    return std::move(msg_);
  }

private:
  ::state_estimator_msgs::msg::LegOdometry msg_;
};

class Init_LegOdometry_lin_vel_rh
{
public:
  explicit Init_LegOdometry_lin_vel_rh(::state_estimator_msgs::msg::LegOdometry & msg)
  : msg_(msg)
  {}
  Init_LegOdometry_base_velocity lin_vel_rh(::state_estimator_msgs::msg::LegOdometry::_lin_vel_rh_type arg)
  {
    msg_.lin_vel_rh = std::move(arg);
    return Init_LegOdometry_base_velocity(msg_);
  }

private:
  ::state_estimator_msgs::msg::LegOdometry msg_;
};

class Init_LegOdometry_lin_vel_lh
{
public:
  explicit Init_LegOdometry_lin_vel_lh(::state_estimator_msgs::msg::LegOdometry & msg)
  : msg_(msg)
  {}
  Init_LegOdometry_lin_vel_rh lin_vel_lh(::state_estimator_msgs::msg::LegOdometry::_lin_vel_lh_type arg)
  {
    msg_.lin_vel_lh = std::move(arg);
    return Init_LegOdometry_lin_vel_rh(msg_);
  }

private:
  ::state_estimator_msgs::msg::LegOdometry msg_;
};

class Init_LegOdometry_lin_vel_rf
{
public:
  explicit Init_LegOdometry_lin_vel_rf(::state_estimator_msgs::msg::LegOdometry & msg)
  : msg_(msg)
  {}
  Init_LegOdometry_lin_vel_lh lin_vel_rf(::state_estimator_msgs::msg::LegOdometry::_lin_vel_rf_type arg)
  {
    msg_.lin_vel_rf = std::move(arg);
    return Init_LegOdometry_lin_vel_lh(msg_);
  }

private:
  ::state_estimator_msgs::msg::LegOdometry msg_;
};

class Init_LegOdometry_lin_vel_lf
{
public:
  explicit Init_LegOdometry_lin_vel_lf(::state_estimator_msgs::msg::LegOdometry & msg)
  : msg_(msg)
  {}
  Init_LegOdometry_lin_vel_rf lin_vel_lf(::state_estimator_msgs::msg::LegOdometry::_lin_vel_lf_type arg)
  {
    msg_.lin_vel_lf = std::move(arg);
    return Init_LegOdometry_lin_vel_rf(msg_);
  }

private:
  ::state_estimator_msgs::msg::LegOdometry msg_;
};

class Init_LegOdometry_header
{
public:
  Init_LegOdometry_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_LegOdometry_lin_vel_lf header(::state_estimator_msgs::msg::LegOdometry::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_LegOdometry_lin_vel_lf(msg_);
  }

private:
  ::state_estimator_msgs::msg::LegOdometry msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::state_estimator_msgs::msg::LegOdometry>()
{
  return state_estimator_msgs::msg::builder::Init_LegOdometry_header();
}

}  // namespace state_estimator_msgs

#endif  // STATE_ESTIMATOR_MSGS__MSG__DETAIL__LEG_ODOMETRY__BUILDER_HPP_
