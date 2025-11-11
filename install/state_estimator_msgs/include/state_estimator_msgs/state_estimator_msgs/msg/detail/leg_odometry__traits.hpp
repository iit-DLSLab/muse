// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from state_estimator_msgs:msg/LegOdometry.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "state_estimator_msgs/msg/leg_odometry.hpp"


#ifndef STATE_ESTIMATOR_MSGS__MSG__DETAIL__LEG_ODOMETRY__TRAITS_HPP_
#define STATE_ESTIMATOR_MSGS__MSG__DETAIL__LEG_ODOMETRY__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "state_estimator_msgs/msg/detail/leg_odometry__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace state_estimator_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const LegOdometry & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: lin_vel_lf
  {
    if (msg.lin_vel_lf.size() == 0) {
      out << "lin_vel_lf: []";
    } else {
      out << "lin_vel_lf: [";
      size_t pending_items = msg.lin_vel_lf.size();
      for (auto item : msg.lin_vel_lf) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: lin_vel_rf
  {
    if (msg.lin_vel_rf.size() == 0) {
      out << "lin_vel_rf: []";
    } else {
      out << "lin_vel_rf: [";
      size_t pending_items = msg.lin_vel_rf.size();
      for (auto item : msg.lin_vel_rf) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: lin_vel_lh
  {
    if (msg.lin_vel_lh.size() == 0) {
      out << "lin_vel_lh: []";
    } else {
      out << "lin_vel_lh: [";
      size_t pending_items = msg.lin_vel_lh.size();
      for (auto item : msg.lin_vel_lh) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: lin_vel_rh
  {
    if (msg.lin_vel_rh.size() == 0) {
      out << "lin_vel_rh: []";
    } else {
      out << "lin_vel_rh: [";
      size_t pending_items = msg.lin_vel_rh.size();
      for (auto item : msg.lin_vel_rh) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: base_velocity
  {
    if (msg.base_velocity.size() == 0) {
      out << "base_velocity: []";
    } else {
      out << "base_velocity: [";
      size_t pending_items = msg.base_velocity.size();
      for (auto item : msg.base_velocity) {
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
  const LegOdometry & msg,
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

  // member: lin_vel_lf
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.lin_vel_lf.size() == 0) {
      out << "lin_vel_lf: []\n";
    } else {
      out << "lin_vel_lf:\n";
      for (auto item : msg.lin_vel_lf) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: lin_vel_rf
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.lin_vel_rf.size() == 0) {
      out << "lin_vel_rf: []\n";
    } else {
      out << "lin_vel_rf:\n";
      for (auto item : msg.lin_vel_rf) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: lin_vel_lh
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.lin_vel_lh.size() == 0) {
      out << "lin_vel_lh: []\n";
    } else {
      out << "lin_vel_lh:\n";
      for (auto item : msg.lin_vel_lh) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: lin_vel_rh
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.lin_vel_rh.size() == 0) {
      out << "lin_vel_rh: []\n";
    } else {
      out << "lin_vel_rh:\n";
      for (auto item : msg.lin_vel_rh) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: base_velocity
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.base_velocity.size() == 0) {
      out << "base_velocity: []\n";
    } else {
      out << "base_velocity:\n";
      for (auto item : msg.base_velocity) {
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

inline std::string to_yaml(const LegOdometry & msg, bool use_flow_style = false)
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
inline const char * data_type<state_estimator_msgs::msg::LegOdometry>()
{
  return "state_estimator_msgs::msg::LegOdometry";
}

template<>
inline const char * name<state_estimator_msgs::msg::LegOdometry>()
{
  return "state_estimator_msgs/msg/LegOdometry";
}

template<>
struct has_fixed_size<state_estimator_msgs::msg::LegOdometry>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<state_estimator_msgs::msg::LegOdometry>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<state_estimator_msgs::msg::LegOdometry>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // STATE_ESTIMATOR_MSGS__MSG__DETAIL__LEG_ODOMETRY__TRAITS_HPP_
