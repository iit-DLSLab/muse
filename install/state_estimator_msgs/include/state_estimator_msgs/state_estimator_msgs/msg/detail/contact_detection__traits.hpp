// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from state_estimator_msgs:msg/ContactDetection.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "state_estimator_msgs/msg/contact_detection.hpp"


#ifndef STATE_ESTIMATOR_MSGS__MSG__DETAIL__CONTACT_DETECTION__TRAITS_HPP_
#define STATE_ESTIMATOR_MSGS__MSG__DETAIL__CONTACT_DETECTION__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "state_estimator_msgs/msg/detail/contact_detection__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace state_estimator_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const ContactDetection & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: stance_lf
  {
    out << "stance_lf: ";
    rosidl_generator_traits::value_to_yaml(msg.stance_lf, out);
    out << ", ";
  }

  // member: stance_rf
  {
    out << "stance_rf: ";
    rosidl_generator_traits::value_to_yaml(msg.stance_rf, out);
    out << ", ";
  }

  // member: stance_lh
  {
    out << "stance_lh: ";
    rosidl_generator_traits::value_to_yaml(msg.stance_lh, out);
    out << ", ";
  }

  // member: stance_rh
  {
    out << "stance_rh: ";
    rosidl_generator_traits::value_to_yaml(msg.stance_rh, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ContactDetection & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: stance_lf
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "stance_lf: ";
    rosidl_generator_traits::value_to_yaml(msg.stance_lf, out);
    out << "\n";
  }

  // member: stance_rf
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "stance_rf: ";
    rosidl_generator_traits::value_to_yaml(msg.stance_rf, out);
    out << "\n";
  }

  // member: stance_lh
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "stance_lh: ";
    rosidl_generator_traits::value_to_yaml(msg.stance_lh, out);
    out << "\n";
  }

  // member: stance_rh
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "stance_rh: ";
    rosidl_generator_traits::value_to_yaml(msg.stance_rh, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ContactDetection & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace state_estimator_msgs

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<state_estimator_msgs::msg::ContactDetection>()
{
  return "state_estimator_msgs::msg::ContactDetection";
}

template<>
inline const char * name<state_estimator_msgs::msg::ContactDetection>()
{
  return "state_estimator_msgs/msg/ContactDetection";
}

template<>
struct has_fixed_size<state_estimator_msgs::msg::ContactDetection>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<state_estimator_msgs::msg::ContactDetection>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<state_estimator_msgs::msg::ContactDetection>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // STATE_ESTIMATOR_MSGS__MSG__DETAIL__CONTACT_DETECTION__TRAITS_HPP_
