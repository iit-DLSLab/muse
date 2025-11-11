// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from state_estimator_msgs:msg/SensorFusion.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "state_estimator_msgs/msg/sensor_fusion.hpp"


#ifndef STATE_ESTIMATOR_MSGS__MSG__DETAIL__SENSOR_FUSION__BUILDER_HPP_
#define STATE_ESTIMATOR_MSGS__MSG__DETAIL__SENSOR_FUSION__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "state_estimator_msgs/msg/detail/sensor_fusion__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace state_estimator_msgs
{

namespace msg
{

namespace builder
{

class Init_SensorFusion_linear_velocity
{
public:
  explicit Init_SensorFusion_linear_velocity(::state_estimator_msgs::msg::SensorFusion & msg)
  : msg_(msg)
  {}
  ::state_estimator_msgs::msg::SensorFusion linear_velocity(::state_estimator_msgs::msg::SensorFusion::_linear_velocity_type arg)
  {
    msg_.linear_velocity = std::move(arg);
    return std::move(msg_);
  }

private:
  ::state_estimator_msgs::msg::SensorFusion msg_;
};

class Init_SensorFusion_position
{
public:
  explicit Init_SensorFusion_position(::state_estimator_msgs::msg::SensorFusion & msg)
  : msg_(msg)
  {}
  Init_SensorFusion_linear_velocity position(::state_estimator_msgs::msg::SensorFusion::_position_type arg)
  {
    msg_.position = std::move(arg);
    return Init_SensorFusion_linear_velocity(msg_);
  }

private:
  ::state_estimator_msgs::msg::SensorFusion msg_;
};

class Init_SensorFusion_header
{
public:
  Init_SensorFusion_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SensorFusion_position header(::state_estimator_msgs::msg::SensorFusion::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_SensorFusion_position(msg_);
  }

private:
  ::state_estimator_msgs::msg::SensorFusion msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::state_estimator_msgs::msg::SensorFusion>()
{
  return state_estimator_msgs::msg::builder::Init_SensorFusion_header();
}

}  // namespace state_estimator_msgs

#endif  // STATE_ESTIMATOR_MSGS__MSG__DETAIL__SENSOR_FUSION__BUILDER_HPP_
