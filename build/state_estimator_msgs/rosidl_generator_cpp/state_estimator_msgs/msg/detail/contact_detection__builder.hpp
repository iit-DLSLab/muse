// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from state_estimator_msgs:msg/ContactDetection.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "state_estimator_msgs/msg/contact_detection.hpp"


#ifndef STATE_ESTIMATOR_MSGS__MSG__DETAIL__CONTACT_DETECTION__BUILDER_HPP_
#define STATE_ESTIMATOR_MSGS__MSG__DETAIL__CONTACT_DETECTION__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "state_estimator_msgs/msg/detail/contact_detection__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace state_estimator_msgs
{

namespace msg
{

namespace builder
{

class Init_ContactDetection_stance_rh
{
public:
  explicit Init_ContactDetection_stance_rh(::state_estimator_msgs::msg::ContactDetection & msg)
  : msg_(msg)
  {}
  ::state_estimator_msgs::msg::ContactDetection stance_rh(::state_estimator_msgs::msg::ContactDetection::_stance_rh_type arg)
  {
    msg_.stance_rh = std::move(arg);
    return std::move(msg_);
  }

private:
  ::state_estimator_msgs::msg::ContactDetection msg_;
};

class Init_ContactDetection_stance_lh
{
public:
  explicit Init_ContactDetection_stance_lh(::state_estimator_msgs::msg::ContactDetection & msg)
  : msg_(msg)
  {}
  Init_ContactDetection_stance_rh stance_lh(::state_estimator_msgs::msg::ContactDetection::_stance_lh_type arg)
  {
    msg_.stance_lh = std::move(arg);
    return Init_ContactDetection_stance_rh(msg_);
  }

private:
  ::state_estimator_msgs::msg::ContactDetection msg_;
};

class Init_ContactDetection_stance_rf
{
public:
  explicit Init_ContactDetection_stance_rf(::state_estimator_msgs::msg::ContactDetection & msg)
  : msg_(msg)
  {}
  Init_ContactDetection_stance_lh stance_rf(::state_estimator_msgs::msg::ContactDetection::_stance_rf_type arg)
  {
    msg_.stance_rf = std::move(arg);
    return Init_ContactDetection_stance_lh(msg_);
  }

private:
  ::state_estimator_msgs::msg::ContactDetection msg_;
};

class Init_ContactDetection_stance_lf
{
public:
  explicit Init_ContactDetection_stance_lf(::state_estimator_msgs::msg::ContactDetection & msg)
  : msg_(msg)
  {}
  Init_ContactDetection_stance_rf stance_lf(::state_estimator_msgs::msg::ContactDetection::_stance_lf_type arg)
  {
    msg_.stance_lf = std::move(arg);
    return Init_ContactDetection_stance_rf(msg_);
  }

private:
  ::state_estimator_msgs::msg::ContactDetection msg_;
};

class Init_ContactDetection_header
{
public:
  Init_ContactDetection_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ContactDetection_stance_lf header(::state_estimator_msgs::msg::ContactDetection::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_ContactDetection_stance_lf(msg_);
  }

private:
  ::state_estimator_msgs::msg::ContactDetection msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::state_estimator_msgs::msg::ContactDetection>()
{
  return state_estimator_msgs::msg::builder::Init_ContactDetection_header();
}

}  // namespace state_estimator_msgs

#endif  // STATE_ESTIMATOR_MSGS__MSG__DETAIL__CONTACT_DETECTION__BUILDER_HPP_
