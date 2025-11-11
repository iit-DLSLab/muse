// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from state_estimator_msgs:msg/LegOdometry.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "state_estimator_msgs/msg/leg_odometry.hpp"


#ifndef STATE_ESTIMATOR_MSGS__MSG__DETAIL__LEG_ODOMETRY__STRUCT_HPP_
#define STATE_ESTIMATOR_MSGS__MSG__DETAIL__LEG_ODOMETRY__STRUCT_HPP_

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
# define DEPRECATED__state_estimator_msgs__msg__LegOdometry __attribute__((deprecated))
#else
# define DEPRECATED__state_estimator_msgs__msg__LegOdometry __declspec(deprecated)
#endif

namespace state_estimator_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct LegOdometry_
{
  using Type = LegOdometry_<ContainerAllocator>;

  explicit LegOdometry_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      std::fill(this->lin_vel_lf.begin(), this->lin_vel_lf.end(), 0.0);
      std::fill(this->lin_vel_rf.begin(), this->lin_vel_rf.end(), 0.0);
      std::fill(this->lin_vel_lh.begin(), this->lin_vel_lh.end(), 0.0);
      std::fill(this->lin_vel_rh.begin(), this->lin_vel_rh.end(), 0.0);
      std::fill(this->base_velocity.begin(), this->base_velocity.end(), 0.0);
    }
  }

  explicit LegOdometry_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    lin_vel_lf(_alloc),
    lin_vel_rf(_alloc),
    lin_vel_lh(_alloc),
    lin_vel_rh(_alloc),
    base_velocity(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      std::fill(this->lin_vel_lf.begin(), this->lin_vel_lf.end(), 0.0);
      std::fill(this->lin_vel_rf.begin(), this->lin_vel_rf.end(), 0.0);
      std::fill(this->lin_vel_lh.begin(), this->lin_vel_lh.end(), 0.0);
      std::fill(this->lin_vel_rh.begin(), this->lin_vel_rh.end(), 0.0);
      std::fill(this->base_velocity.begin(), this->base_velocity.end(), 0.0);
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _lin_vel_lf_type =
    std::array<double, 3>;
  _lin_vel_lf_type lin_vel_lf;
  using _lin_vel_rf_type =
    std::array<double, 3>;
  _lin_vel_rf_type lin_vel_rf;
  using _lin_vel_lh_type =
    std::array<double, 3>;
  _lin_vel_lh_type lin_vel_lh;
  using _lin_vel_rh_type =
    std::array<double, 3>;
  _lin_vel_rh_type lin_vel_rh;
  using _base_velocity_type =
    std::array<double, 3>;
  _base_velocity_type base_velocity;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__lin_vel_lf(
    const std::array<double, 3> & _arg)
  {
    this->lin_vel_lf = _arg;
    return *this;
  }
  Type & set__lin_vel_rf(
    const std::array<double, 3> & _arg)
  {
    this->lin_vel_rf = _arg;
    return *this;
  }
  Type & set__lin_vel_lh(
    const std::array<double, 3> & _arg)
  {
    this->lin_vel_lh = _arg;
    return *this;
  }
  Type & set__lin_vel_rh(
    const std::array<double, 3> & _arg)
  {
    this->lin_vel_rh = _arg;
    return *this;
  }
  Type & set__base_velocity(
    const std::array<double, 3> & _arg)
  {
    this->base_velocity = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    state_estimator_msgs::msg::LegOdometry_<ContainerAllocator> *;
  using ConstRawPtr =
    const state_estimator_msgs::msg::LegOdometry_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<state_estimator_msgs::msg::LegOdometry_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<state_estimator_msgs::msg::LegOdometry_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      state_estimator_msgs::msg::LegOdometry_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<state_estimator_msgs::msg::LegOdometry_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      state_estimator_msgs::msg::LegOdometry_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<state_estimator_msgs::msg::LegOdometry_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<state_estimator_msgs::msg::LegOdometry_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<state_estimator_msgs::msg::LegOdometry_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__state_estimator_msgs__msg__LegOdometry
    std::shared_ptr<state_estimator_msgs::msg::LegOdometry_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__state_estimator_msgs__msg__LegOdometry
    std::shared_ptr<state_estimator_msgs::msg::LegOdometry_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const LegOdometry_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->lin_vel_lf != other.lin_vel_lf) {
      return false;
    }
    if (this->lin_vel_rf != other.lin_vel_rf) {
      return false;
    }
    if (this->lin_vel_lh != other.lin_vel_lh) {
      return false;
    }
    if (this->lin_vel_rh != other.lin_vel_rh) {
      return false;
    }
    if (this->base_velocity != other.base_velocity) {
      return false;
    }
    return true;
  }
  bool operator!=(const LegOdometry_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct LegOdometry_

// alias to use template instance with default allocator
using LegOdometry =
  state_estimator_msgs::msg::LegOdometry_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace state_estimator_msgs

#endif  // STATE_ESTIMATOR_MSGS__MSG__DETAIL__LEG_ODOMETRY__STRUCT_HPP_
