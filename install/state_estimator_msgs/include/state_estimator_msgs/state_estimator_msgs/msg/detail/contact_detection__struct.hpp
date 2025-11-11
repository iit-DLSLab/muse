// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from state_estimator_msgs:msg/ContactDetection.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "state_estimator_msgs/msg/contact_detection.hpp"


#ifndef STATE_ESTIMATOR_MSGS__MSG__DETAIL__CONTACT_DETECTION__STRUCT_HPP_
#define STATE_ESTIMATOR_MSGS__MSG__DETAIL__CONTACT_DETECTION__STRUCT_HPP_

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
# define DEPRECATED__state_estimator_msgs__msg__ContactDetection __attribute__((deprecated))
#else
# define DEPRECATED__state_estimator_msgs__msg__ContactDetection __declspec(deprecated)
#endif

namespace state_estimator_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ContactDetection_
{
  using Type = ContactDetection_<ContainerAllocator>;

  explicit ContactDetection_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->stance_lf = false;
      this->stance_rf = false;
      this->stance_lh = false;
      this->stance_rh = false;
    }
  }

  explicit ContactDetection_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->stance_lf = false;
      this->stance_rf = false;
      this->stance_lh = false;
      this->stance_rh = false;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _stance_lf_type =
    bool;
  _stance_lf_type stance_lf;
  using _stance_rf_type =
    bool;
  _stance_rf_type stance_rf;
  using _stance_lh_type =
    bool;
  _stance_lh_type stance_lh;
  using _stance_rh_type =
    bool;
  _stance_rh_type stance_rh;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__stance_lf(
    const bool & _arg)
  {
    this->stance_lf = _arg;
    return *this;
  }
  Type & set__stance_rf(
    const bool & _arg)
  {
    this->stance_rf = _arg;
    return *this;
  }
  Type & set__stance_lh(
    const bool & _arg)
  {
    this->stance_lh = _arg;
    return *this;
  }
  Type & set__stance_rh(
    const bool & _arg)
  {
    this->stance_rh = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    state_estimator_msgs::msg::ContactDetection_<ContainerAllocator> *;
  using ConstRawPtr =
    const state_estimator_msgs::msg::ContactDetection_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<state_estimator_msgs::msg::ContactDetection_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<state_estimator_msgs::msg::ContactDetection_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      state_estimator_msgs::msg::ContactDetection_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<state_estimator_msgs::msg::ContactDetection_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      state_estimator_msgs::msg::ContactDetection_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<state_estimator_msgs::msg::ContactDetection_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<state_estimator_msgs::msg::ContactDetection_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<state_estimator_msgs::msg::ContactDetection_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__state_estimator_msgs__msg__ContactDetection
    std::shared_ptr<state_estimator_msgs::msg::ContactDetection_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__state_estimator_msgs__msg__ContactDetection
    std::shared_ptr<state_estimator_msgs::msg::ContactDetection_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ContactDetection_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->stance_lf != other.stance_lf) {
      return false;
    }
    if (this->stance_rf != other.stance_rf) {
      return false;
    }
    if (this->stance_lh != other.stance_lh) {
      return false;
    }
    if (this->stance_rh != other.stance_rh) {
      return false;
    }
    return true;
  }
  bool operator!=(const ContactDetection_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ContactDetection_

// alias to use template instance with default allocator
using ContactDetection =
  state_estimator_msgs::msg::ContactDetection_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace state_estimator_msgs

#endif  // STATE_ESTIMATOR_MSGS__MSG__DETAIL__CONTACT_DETECTION__STRUCT_HPP_
