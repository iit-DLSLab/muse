// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from state_estimator_msgs:msg/SensorFusion.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "state_estimator_msgs/msg/sensor_fusion.hpp"


#ifndef STATE_ESTIMATOR_MSGS__MSG__DETAIL__SENSOR_FUSION__STRUCT_HPP_
#define STATE_ESTIMATOR_MSGS__MSG__DETAIL__SENSOR_FUSION__STRUCT_HPP_

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
# define DEPRECATED__state_estimator_msgs__msg__SensorFusion __attribute__((deprecated))
#else
# define DEPRECATED__state_estimator_msgs__msg__SensorFusion __declspec(deprecated)
#endif

namespace state_estimator_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct SensorFusion_
{
  using Type = SensorFusion_<ContainerAllocator>;

  explicit SensorFusion_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      std::fill(this->position.begin(), this->position.end(), 0.0);
      std::fill(this->linear_velocity.begin(), this->linear_velocity.end(), 0.0);
    }
  }

  explicit SensorFusion_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    position(_alloc),
    linear_velocity(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      std::fill(this->position.begin(), this->position.end(), 0.0);
      std::fill(this->linear_velocity.begin(), this->linear_velocity.end(), 0.0);
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _position_type =
    std::array<double, 3>;
  _position_type position;
  using _linear_velocity_type =
    std::array<double, 3>;
  _linear_velocity_type linear_velocity;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__position(
    const std::array<double, 3> & _arg)
  {
    this->position = _arg;
    return *this;
  }
  Type & set__linear_velocity(
    const std::array<double, 3> & _arg)
  {
    this->linear_velocity = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    state_estimator_msgs::msg::SensorFusion_<ContainerAllocator> *;
  using ConstRawPtr =
    const state_estimator_msgs::msg::SensorFusion_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<state_estimator_msgs::msg::SensorFusion_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<state_estimator_msgs::msg::SensorFusion_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      state_estimator_msgs::msg::SensorFusion_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<state_estimator_msgs::msg::SensorFusion_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      state_estimator_msgs::msg::SensorFusion_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<state_estimator_msgs::msg::SensorFusion_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<state_estimator_msgs::msg::SensorFusion_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<state_estimator_msgs::msg::SensorFusion_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__state_estimator_msgs__msg__SensorFusion
    std::shared_ptr<state_estimator_msgs::msg::SensorFusion_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__state_estimator_msgs__msg__SensorFusion
    std::shared_ptr<state_estimator_msgs::msg::SensorFusion_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SensorFusion_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->position != other.position) {
      return false;
    }
    if (this->linear_velocity != other.linear_velocity) {
      return false;
    }
    return true;
  }
  bool operator!=(const SensorFusion_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SensorFusion_

// alias to use template instance with default allocator
using SensorFusion =
  state_estimator_msgs::msg::SensorFusion_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace state_estimator_msgs

#endif  // STATE_ESTIMATOR_MSGS__MSG__DETAIL__SENSOR_FUSION__STRUCT_HPP_
