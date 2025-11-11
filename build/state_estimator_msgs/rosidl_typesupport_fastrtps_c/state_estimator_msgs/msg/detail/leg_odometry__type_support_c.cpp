// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from state_estimator_msgs:msg/LegOdometry.idl
// generated code does not contain a copyright notice
#include "state_estimator_msgs/msg/detail/leg_odometry__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <cstddef>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/serialization_helpers.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "state_estimator_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "state_estimator_msgs/msg/detail/leg_odometry__struct.h"
#include "state_estimator_msgs/msg/detail/leg_odometry__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif

#include "std_msgs/msg/detail/header__functions.h"  // header

// forward declare type support functions

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_state_estimator_msgs
bool cdr_serialize_std_msgs__msg__Header(
  const std_msgs__msg__Header * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_state_estimator_msgs
bool cdr_deserialize_std_msgs__msg__Header(
  eprosima::fastcdr::Cdr & cdr,
  std_msgs__msg__Header * ros_message);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_state_estimator_msgs
size_t get_serialized_size_std_msgs__msg__Header(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_state_estimator_msgs
size_t max_serialized_size_std_msgs__msg__Header(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_state_estimator_msgs
bool cdr_serialize_key_std_msgs__msg__Header(
  const std_msgs__msg__Header * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_state_estimator_msgs
size_t get_serialized_size_key_std_msgs__msg__Header(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_state_estimator_msgs
size_t max_serialized_size_key_std_msgs__msg__Header(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_state_estimator_msgs
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, std_msgs, msg, Header)();


using _LegOdometry__ros_msg_type = state_estimator_msgs__msg__LegOdometry;


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_state_estimator_msgs
bool cdr_serialize_state_estimator_msgs__msg__LegOdometry(
  const state_estimator_msgs__msg__LegOdometry * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: header
  {
    cdr_serialize_std_msgs__msg__Header(
      &ros_message->header, cdr);
  }

  // Field name: lin_vel_lf
  {
    size_t size = 3;
    auto array_ptr = ros_message->lin_vel_lf;
    cdr.serialize_array(array_ptr, size);
  }

  // Field name: lin_vel_rf
  {
    size_t size = 3;
    auto array_ptr = ros_message->lin_vel_rf;
    cdr.serialize_array(array_ptr, size);
  }

  // Field name: lin_vel_lh
  {
    size_t size = 3;
    auto array_ptr = ros_message->lin_vel_lh;
    cdr.serialize_array(array_ptr, size);
  }

  // Field name: lin_vel_rh
  {
    size_t size = 3;
    auto array_ptr = ros_message->lin_vel_rh;
    cdr.serialize_array(array_ptr, size);
  }

  // Field name: base_velocity
  {
    size_t size = 3;
    auto array_ptr = ros_message->base_velocity;
    cdr.serialize_array(array_ptr, size);
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_state_estimator_msgs
bool cdr_deserialize_state_estimator_msgs__msg__LegOdometry(
  eprosima::fastcdr::Cdr & cdr,
  state_estimator_msgs__msg__LegOdometry * ros_message)
{
  // Field name: header
  {
    cdr_deserialize_std_msgs__msg__Header(cdr, &ros_message->header);
  }

  // Field name: lin_vel_lf
  {
    size_t size = 3;
    auto array_ptr = ros_message->lin_vel_lf;
    cdr.deserialize_array(array_ptr, size);
  }

  // Field name: lin_vel_rf
  {
    size_t size = 3;
    auto array_ptr = ros_message->lin_vel_rf;
    cdr.deserialize_array(array_ptr, size);
  }

  // Field name: lin_vel_lh
  {
    size_t size = 3;
    auto array_ptr = ros_message->lin_vel_lh;
    cdr.deserialize_array(array_ptr, size);
  }

  // Field name: lin_vel_rh
  {
    size_t size = 3;
    auto array_ptr = ros_message->lin_vel_rh;
    cdr.deserialize_array(array_ptr, size);
  }

  // Field name: base_velocity
  {
    size_t size = 3;
    auto array_ptr = ros_message->base_velocity;
    cdr.deserialize_array(array_ptr, size);
  }

  return true;
}  // NOLINT(readability/fn_size)


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_state_estimator_msgs
size_t get_serialized_size_state_estimator_msgs__msg__LegOdometry(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _LegOdometry__ros_msg_type * ros_message = static_cast<const _LegOdometry__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: header
  current_alignment += get_serialized_size_std_msgs__msg__Header(
    &(ros_message->header), current_alignment);

  // Field name: lin_vel_lf
  {
    size_t array_size = 3;
    auto array_ptr = ros_message->lin_vel_lf;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: lin_vel_rf
  {
    size_t array_size = 3;
    auto array_ptr = ros_message->lin_vel_rf;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: lin_vel_lh
  {
    size_t array_size = 3;
    auto array_ptr = ros_message->lin_vel_lh;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: lin_vel_rh
  {
    size_t array_size = 3;
    auto array_ptr = ros_message->lin_vel_rh;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: base_velocity
  {
    size_t array_size = 3;
    auto array_ptr = ros_message->base_velocity;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_state_estimator_msgs
size_t max_serialized_size_state_estimator_msgs__msg__LegOdometry(
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

  // Field name: header
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_std_msgs__msg__Header(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: lin_vel_lf
  {
    size_t array_size = 3;
    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Field name: lin_vel_rf
  {
    size_t array_size = 3;
    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Field name: lin_vel_lh
  {
    size_t array_size = 3;
    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Field name: lin_vel_rh
  {
    size_t array_size = 3;
    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Field name: base_velocity
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
    using DataType = state_estimator_msgs__msg__LegOdometry;
    is_plain =
      (
      offsetof(DataType, base_velocity) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_state_estimator_msgs
bool cdr_serialize_key_state_estimator_msgs__msg__LegOdometry(
  const state_estimator_msgs__msg__LegOdometry * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: header
  {
    cdr_serialize_key_std_msgs__msg__Header(
      &ros_message->header, cdr);
  }

  // Field name: lin_vel_lf
  {
    size_t size = 3;
    auto array_ptr = ros_message->lin_vel_lf;
    cdr.serialize_array(array_ptr, size);
  }

  // Field name: lin_vel_rf
  {
    size_t size = 3;
    auto array_ptr = ros_message->lin_vel_rf;
    cdr.serialize_array(array_ptr, size);
  }

  // Field name: lin_vel_lh
  {
    size_t size = 3;
    auto array_ptr = ros_message->lin_vel_lh;
    cdr.serialize_array(array_ptr, size);
  }

  // Field name: lin_vel_rh
  {
    size_t size = 3;
    auto array_ptr = ros_message->lin_vel_rh;
    cdr.serialize_array(array_ptr, size);
  }

  // Field name: base_velocity
  {
    size_t size = 3;
    auto array_ptr = ros_message->base_velocity;
    cdr.serialize_array(array_ptr, size);
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_state_estimator_msgs
size_t get_serialized_size_key_state_estimator_msgs__msg__LegOdometry(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _LegOdometry__ros_msg_type * ros_message = static_cast<const _LegOdometry__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;

  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: header
  current_alignment += get_serialized_size_key_std_msgs__msg__Header(
    &(ros_message->header), current_alignment);

  // Field name: lin_vel_lf
  {
    size_t array_size = 3;
    auto array_ptr = ros_message->lin_vel_lf;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: lin_vel_rf
  {
    size_t array_size = 3;
    auto array_ptr = ros_message->lin_vel_rf;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: lin_vel_lh
  {
    size_t array_size = 3;
    auto array_ptr = ros_message->lin_vel_lh;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: lin_vel_rh
  {
    size_t array_size = 3;
    auto array_ptr = ros_message->lin_vel_rh;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: base_velocity
  {
    size_t array_size = 3;
    auto array_ptr = ros_message->base_velocity;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_state_estimator_msgs
size_t max_serialized_size_key_state_estimator_msgs__msg__LegOdometry(
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
  // Field name: header
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_key_std_msgs__msg__Header(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: lin_vel_lf
  {
    size_t array_size = 3;
    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Field name: lin_vel_rf
  {
    size_t array_size = 3;
    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Field name: lin_vel_lh
  {
    size_t array_size = 3;
    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Field name: lin_vel_rh
  {
    size_t array_size = 3;
    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Field name: base_velocity
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
    using DataType = state_estimator_msgs__msg__LegOdometry;
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
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const state_estimator_msgs__msg__LegOdometry * ros_message = static_cast<const state_estimator_msgs__msg__LegOdometry *>(untyped_ros_message);
  (void)ros_message;
  return cdr_serialize_state_estimator_msgs__msg__LegOdometry(ros_message, cdr);
}

static bool _LegOdometry__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  state_estimator_msgs__msg__LegOdometry * ros_message = static_cast<state_estimator_msgs__msg__LegOdometry *>(untyped_ros_message);
  (void)ros_message;
  return cdr_deserialize_state_estimator_msgs__msg__LegOdometry(cdr, ros_message);
}

static uint32_t _LegOdometry__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_state_estimator_msgs__msg__LegOdometry(
      untyped_ros_message, 0));
}

static size_t _LegOdometry__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_state_estimator_msgs__msg__LegOdometry(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_LegOdometry = {
  "state_estimator_msgs::msg",
  "LegOdometry",
  _LegOdometry__cdr_serialize,
  _LegOdometry__cdr_deserialize,
  _LegOdometry__get_serialized_size,
  _LegOdometry__max_serialized_size,
  nullptr
};

static rosidl_message_type_support_t _LegOdometry__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_LegOdometry,
  get_message_typesupport_handle_function,
  &state_estimator_msgs__msg__LegOdometry__get_type_hash,
  &state_estimator_msgs__msg__LegOdometry__get_type_description,
  &state_estimator_msgs__msg__LegOdometry__get_type_description_sources,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, state_estimator_msgs, msg, LegOdometry)() {
  return &_LegOdometry__type_support;
}

#if defined(__cplusplus)
}
#endif
