// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from state_estimator_msgs:msg/LegOdometry.idl
// generated code does not contain a copyright notice
#include "state_estimator_msgs/msg/detail/leg_odometry__rosidl_typesupport_fastrtps_cpp.hpp"
#include "state_estimator_msgs/msg/detail/leg_odometry__functions.h"
#include "state_estimator_msgs/msg/detail/leg_odometry__struct.hpp"

#include <cstddef>
#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/serialization_helpers.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions
namespace std_msgs
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const std_msgs::msg::Header &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  std_msgs::msg::Header &);
size_t get_serialized_size(
  const std_msgs::msg::Header &,
  size_t current_alignment);
size_t
max_serialized_size_Header(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);
bool cdr_serialize_key(
  const std_msgs::msg::Header &,
  eprosima::fastcdr::Cdr &);
size_t get_serialized_size_key(
  const std_msgs::msg::Header &,
  size_t current_alignment);
size_t
max_serialized_size_key_Header(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace std_msgs


namespace state_estimator_msgs
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{


bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_state_estimator_msgs
cdr_serialize(
  const state_estimator_msgs::msg::LegOdometry & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: header
  std_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.header,
    cdr);

  // Member: lin_vel_lf
  {
    cdr << ros_message.lin_vel_lf;
  }

  // Member: lin_vel_rf
  {
    cdr << ros_message.lin_vel_rf;
  }

  // Member: lin_vel_lh
  {
    cdr << ros_message.lin_vel_lh;
  }

  // Member: lin_vel_rh
  {
    cdr << ros_message.lin_vel_rh;
  }

  // Member: base_velocity
  {
    cdr << ros_message.base_velocity;
  }

  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_state_estimator_msgs
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  state_estimator_msgs::msg::LegOdometry & ros_message)
{
  // Member: header
  std_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.header);

  // Member: lin_vel_lf
  {
    cdr >> ros_message.lin_vel_lf;
  }

  // Member: lin_vel_rf
  {
    cdr >> ros_message.lin_vel_rf;
  }

  // Member: lin_vel_lh
  {
    cdr >> ros_message.lin_vel_lh;
  }

  // Member: lin_vel_rh
  {
    cdr >> ros_message.lin_vel_rh;
  }

  // Member: base_velocity
  {
    cdr >> ros_message.base_velocity;
  }

  return true;
}  // NOLINT(readability/fn_size)


size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_state_estimator_msgs
get_serialized_size(
  const state_estimator_msgs::msg::LegOdometry & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: header
  current_alignment +=
    std_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.header, current_alignment);

  // Member: lin_vel_lf
  {
    size_t array_size = 3;
    size_t item_size = sizeof(ros_message.lin_vel_lf[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: lin_vel_rf
  {
    size_t array_size = 3;
    size_t item_size = sizeof(ros_message.lin_vel_rf[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: lin_vel_lh
  {
    size_t array_size = 3;
    size_t item_size = sizeof(ros_message.lin_vel_lh[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: lin_vel_rh
  {
    size_t array_size = 3;
    size_t item_size = sizeof(ros_message.lin_vel_rh[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: base_velocity
  {
    size_t array_size = 3;
    size_t item_size = sizeof(ros_message.base_velocity[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}


size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_state_estimator_msgs
max_serialized_size_LegOdometry(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // Member: header
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size =
        std_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_Header(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }
  // Member: lin_vel_lf
  {
    size_t array_size = 3;
    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // Member: lin_vel_rf
  {
    size_t array_size = 3;
    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // Member: lin_vel_lh
  {
    size_t array_size = 3;
    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // Member: lin_vel_rh
  {
    size_t array_size = 3;
    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // Member: base_velocity
  {
    size_t array_size = 3;
    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = state_estimator_msgs::msg::LegOdometry;
    is_plain =
      (
      offsetof(DataType, base_velocity) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_state_estimator_msgs
cdr_serialize_key(
  const state_estimator_msgs::msg::LegOdometry & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: header
  std_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize_key(
    ros_message.header,
    cdr);

  // Member: lin_vel_lf
  {
    cdr << ros_message.lin_vel_lf;
  }

  // Member: lin_vel_rf
  {
    cdr << ros_message.lin_vel_rf;
  }

  // Member: lin_vel_lh
  {
    cdr << ros_message.lin_vel_lh;
  }

  // Member: lin_vel_rh
  {
    cdr << ros_message.lin_vel_rh;
  }

  // Member: base_velocity
  {
    cdr << ros_message.base_velocity;
  }

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_state_estimator_msgs
get_serialized_size_key(
  const state_estimator_msgs::msg::LegOdometry & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: header
  current_alignment +=
    std_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size_key(
    ros_message.header, current_alignment);

  // Member: lin_vel_lf
  {
    size_t array_size = 3;
    size_t item_size = sizeof(ros_message.lin_vel_lf[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: lin_vel_rf
  {
    size_t array_size = 3;
    size_t item_size = sizeof(ros_message.lin_vel_rf[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: lin_vel_lh
  {
    size_t array_size = 3;
    size_t item_size = sizeof(ros_message.lin_vel_lh[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: lin_vel_rh
  {
    size_t array_size = 3;
    size_t item_size = sizeof(ros_message.lin_vel_rh[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: base_velocity
  {
    size_t array_size = 3;
    size_t item_size = sizeof(ros_message.base_velocity[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_state_estimator_msgs
max_serialized_size_key_LegOdometry(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // Member: header
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size =
        std_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_key_Header(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Member: lin_vel_lf
  {
    size_t array_size = 3;
    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: lin_vel_rf
  {
    size_t array_size = 3;
    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: lin_vel_lh
  {
    size_t array_size = 3;
    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: lin_vel_rh
  {
    size_t array_size = 3;
    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: base_velocity
  {
    size_t array_size = 3;
    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = state_estimator_msgs::msg::LegOdometry;
    is_plain =
      (
      offsetof(DataType, base_velocity) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}


static bool _LegOdometry__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const state_estimator_msgs::msg::LegOdometry *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _LegOdometry__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<state_estimator_msgs::msg::LegOdometry *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _LegOdometry__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const state_estimator_msgs::msg::LegOdometry *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _LegOdometry__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_LegOdometry(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _LegOdometry__callbacks = {
  "state_estimator_msgs::msg",
  "LegOdometry",
  _LegOdometry__cdr_serialize,
  _LegOdometry__cdr_deserialize,
  _LegOdometry__get_serialized_size,
  _LegOdometry__max_serialized_size,
  nullptr
};

static rosidl_message_type_support_t _LegOdometry__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_LegOdometry__callbacks,
  get_message_typesupport_handle_function,
  &state_estimator_msgs__msg__LegOdometry__get_type_hash,
  &state_estimator_msgs__msg__LegOdometry__get_type_description,
  &state_estimator_msgs__msg__LegOdometry__get_type_description_sources,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace state_estimator_msgs

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_state_estimator_msgs
const rosidl_message_type_support_t *
get_message_type_support_handle<state_estimator_msgs::msg::LegOdometry>()
{
  return &state_estimator_msgs::msg::typesupport_fastrtps_cpp::_LegOdometry__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, state_estimator_msgs, msg, LegOdometry)() {
  return &state_estimator_msgs::msg::typesupport_fastrtps_cpp::_LegOdometry__handle;
}

#ifdef __cplusplus
}
#endif
