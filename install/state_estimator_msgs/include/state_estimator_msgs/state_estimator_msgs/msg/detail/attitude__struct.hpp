// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from state_estimator_msgs:msg/Attitude.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "state_estimator_msgs/msg/attitude.hpp"


#ifndef STATE_ESTIMATOR_MSGS__MSG__DETAIL__ATTITUDE__STRUCT_HPP_
#define STATE_ESTIMATOR_MSGS__MSG__DETAIL__ATTITUDE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__state_estimator_msgs__msg__Attitude __attribute__((deprecated))
#else
# define DEPRECATED__state_estimator_msgs__msg__Attitude __declspec(deprecated)
#endif

namespace state_estimator_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Attitude_
{
  using Type = Attitude_<ContainerAllocator>;

  explicit Attitude_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      std::fill(this->quaternion.begin(), this->quaternion.end(), 0.0);
      this->roll_deg = 0.0;
      this->pitch_deg = 0.0;
      this->yaw_deg = 0.0;
      std::fill(this->angular_velocity.begin(), this->angular_velocity.end(), 0.0);
    }
  }

  explicit Attitude_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    quaternion(_alloc),
    angular_velocity(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      std::fill(this->quaternion.begin(), this->quaternion.end(), 0.0);
      this->roll_deg = 0.0;
      this->pitch_deg = 0.0;
      this->yaw_deg = 0.0;
      std::fill(this->angular_velocity.begin(), this->angular_velocity.end(), 0.0);
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _quaternion_type =
    std::array<double, 4>;
  _quaternion_type quaternion;
  using _roll_deg_type =
    double;
  _roll_deg_type roll_deg;
  using _pitch_deg_type =
    double;
  _pitch_deg_type pitch_deg;
  using _yaw_deg_type =
    double;
  _yaw_deg_type yaw_deg;
  using _angular_velocity_type =
    std::array<double, 3>;
  _angular_velocity_type angular_velocity;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__quaternion(
    const std::array<double, 4> & _arg)
  {
    this->quaternion = _arg;
    return *this;
  }
  Type & set__roll_deg(
    const double & _arg)
  {
    this->roll_deg = _arg;
    return *this;
  }
  Type & set__pitch_deg(
    const double & _arg)
  {
    this->pitch_deg = _arg;
    return *this;
  }
  Type & set__yaw_deg(
    const double & _arg)
  {
    this->yaw_deg = _arg;
    return *this;
  }
  Type & set__angular_velocity(
    const std::array<double, 3> & _arg)
  {
    this->angular_velocity = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    state_estimator_msgs::msg::Attitude_<ContainerAllocator> *;
  using ConstRawPtr =
    const state_estimator_msgs::msg::Attitude_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<state_estimator_msgs::msg::Attitude_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<state_estimator_msgs::msg::Attitude_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      state_estimator_msgs::msg::Attitude_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<state_estimator_msgs::msg::Attitude_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      state_estimator_msgs::msg::Attitude_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<state_estimator_msgs::msg::Attitude_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<state_estimator_msgs::msg::Attitude_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<state_estimator_msgs::msg::Attitude_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__state_estimator_msgs__msg__Attitude
    std::shared_ptr<state_estimator_msgs::msg::Attitude_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__state_estimator_msgs__msg__Attitude
    std::shared_ptr<state_estimator_msgs::msg::Attitude_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Attitude_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->quaternion != other.quaternion) {
      return false;
    }
    if (this->roll_deg != other.roll_deg) {
      return false;
    }
    if (this->pitch_deg != other.pitch_deg) {
      return false;
    }
    if (this->yaw_deg != other.yaw_deg) {
      return false;
    }
    if (this->angular_velocity != other.angular_velocity) {
      return false;
    }
    return true;
  }
  bool operator!=(const Attitude_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Attitude_

// alias to use template instance with default allocator
using Attitude =
  state_estimator_msgs::msg::Attitude_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace state_estimator_msgs

#endif  // STATE_ESTIMATOR_MSGS__MSG__DETAIL__ATTITUDE__STRUCT_HPP_
