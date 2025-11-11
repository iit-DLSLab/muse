// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from state_estimator_msgs:msg/SensorFusion.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "state_estimator_msgs/msg/sensor_fusion.hpp"


#ifndef STATE_ESTIMATOR_MSGS__MSG__DETAIL__SENSOR_FUSION__TRAITS_HPP_
#define STATE_ESTIMATOR_MSGS__MSG__DETAIL__SENSOR_FUSION__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "state_estimator_msgs/msg/detail/sensor_fusion__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace state_estimator_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const SensorFusion & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: position
  {
    if (msg.position.size() == 0) {
      out << "position: []";
    } else {
      out << "position: [";
      size_t pending_items = msg.position.size();
      for (auto item : msg.position) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: linear_velocity
  {
    if (msg.linear_velocity.size() == 0) {
      out << "linear_velocity: []";
    } else {
      out << "linear_velocity: [";
      size_t pending_items = msg.linear_velocity.size();
      for (auto item : msg.linear_velocity) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SensorFusion & msg,
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

  // member: position
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.position.size() == 0) {
      out << "position: []\n";
    } else {
      out << "position:\n";
      for (auto item : msg.position) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: linear_velocity
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.linear_velocity.size() == 0) {
      out << "linear_velocity: []\n";
    } else {
      out << "linear_velocity:\n";
      for (auto item : msg.linear_velocity) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SensorFusion & msg, bool use_flow_style = false)
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
inline const char * data_type<state_estimator_msgs::msg::SensorFusion>()
{
  return "state_estimator_msgs::msg::SensorFusion";
}

template<>
inline const char * name<state_estimator_msgs::msg::SensorFusion>()
{
  return "state_estimator_msgs/msg/SensorFusion";
}

template<>
struct has_fixed_size<state_estimator_msgs::msg::SensorFusion>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<state_estimator_msgs::msg::SensorFusion>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<state_estimator_msgs::msg::SensorFusion>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // STATE_ESTIMATOR_MSGS__MSG__DETAIL__SENSOR_FUSION__TRAITS_HPP_
