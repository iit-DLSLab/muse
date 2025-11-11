// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from state_estimator_msgs:msg/Attitude.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "state_estimator_msgs/msg/attitude.hpp"


#ifndef STATE_ESTIMATOR_MSGS__MSG__DETAIL__ATTITUDE__TRAITS_HPP_
#define STATE_ESTIMATOR_MSGS__MSG__DETAIL__ATTITUDE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "state_estimator_msgs/msg/detail/attitude__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace state_estimator_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const Attitude & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: quaternion
  {
    if (msg.quaternion.size() == 0) {
      out << "quaternion: []";
    } else {
      out << "quaternion: [";
      size_t pending_items = msg.quaternion.size();
      for (auto item : msg.quaternion) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: roll_deg
  {
    out << "roll_deg: ";
    rosidl_generator_traits::value_to_yaml(msg.roll_deg, out);
    out << ", ";
  }

  // member: pitch_deg
  {
    out << "pitch_deg: ";
    rosidl_generator_traits::value_to_yaml(msg.pitch_deg, out);
    out << ", ";
  }

  // member: yaw_deg
  {
    out << "yaw_deg: ";
    rosidl_generator_traits::value_to_yaml(msg.yaw_deg, out);
    out << ", ";
  }

  // member: angular_velocity
  {
    if (msg.angular_velocity.size() == 0) {
      out << "angular_velocity: []";
    } else {
      out << "angular_velocity: [";
      size_t pending_items = msg.angular_velocity.size();
      for (auto item : msg.angular_velocity) {
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
  const Attitude & msg,
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

  // member: quaternion
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.quaternion.size() == 0) {
      out << "quaternion: []\n";
    } else {
      out << "quaternion:\n";
      for (auto item : msg.quaternion) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: roll_deg
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "roll_deg: ";
    rosidl_generator_traits::value_to_yaml(msg.roll_deg, out);
    out << "\n";
  }

  // member: pitch_deg
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pitch_deg: ";
    rosidl_generator_traits::value_to_yaml(msg.pitch_deg, out);
    out << "\n";
  }

  // member: yaw_deg
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "yaw_deg: ";
    rosidl_generator_traits::value_to_yaml(msg.yaw_deg, out);
    out << "\n";
  }

  // member: angular_velocity
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.angular_velocity.size() == 0) {
      out << "angular_velocity: []\n";
    } else {
      out << "angular_velocity:\n";
      for (auto item : msg.angular_velocity) {
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

inline std::string to_yaml(const Attitude & msg, bool use_flow_style = false)
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
inline const char * data_type<state_estimator_msgs::msg::Attitude>()
{
  return "state_estimator_msgs::msg::Attitude";
}

template<>
inline const char * name<state_estimator_msgs::msg::Attitude>()
{
  return "state_estimator_msgs/msg/Attitude";
}

template<>
struct has_fixed_size<state_estimator_msgs::msg::Attitude>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<state_estimator_msgs::msg::Attitude>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<state_estimator_msgs::msg::Attitude>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // STATE_ESTIMATOR_MSGS__MSG__DETAIL__ATTITUDE__TRAITS_HPP_
