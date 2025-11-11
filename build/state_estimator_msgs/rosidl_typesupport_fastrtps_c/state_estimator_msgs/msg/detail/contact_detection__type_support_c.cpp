// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from state_estimator_msgs:msg/ContactDetection.idl
// generated code does not contain a copyright notice
#include "state_estimator_msgs/msg/detail/contact_detection__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <cstddef>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/serialization_helpers.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "state_estimator_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "state_estimator_msgs/msg/detail/contact_detection__struct.h"
#include "state_estimator_msgs/msg/detail/contact_detection__functions.h"
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


using _ContactDetection__ros_msg_type = state_estimator_msgs__msg__ContactDetection;


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_state_estimator_msgs
bool cdr_serialize_state_estimator_msgs__msg__ContactDetection(
  const state_estimator_msgs__msg__ContactDetection * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: header
  {
    cdr_serialize_std_msgs__msg__Header(
      &ros_message->header, cdr);
  }

  // Field name: stance_lf
  {
    cdr << (ros_message->stance_lf ? true : false);
  }

  // Field name: stance_rf
  {
    cdr << (ros_message->stance_rf ? true : false);
  }

  // Field name: stance_lh
  {
    cdr << (ros_message->stance_lh ? true : false);
  }

  // Field name: stance_rh
  {
    cdr << (ros_message->stance_rh ? true : false);
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_state_estimator_msgs
bool cdr_deserialize_state_estimator_msgs__msg__ContactDetection(
  eprosima::fastcdr::Cdr & cdr,
  state_estimator_msgs__msg__ContactDetection * ros_message)
{
  // Field name: header
  {
    cdr_deserialize_std_msgs__msg__Header(cdr, &ros_message->header);
  }

  // Field name: stance_lf
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->stance_lf = tmp ? true : false;
  }

  // Field name: stance_rf
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->stance_rf = tmp ? true : false;
  }

  // Field name: stance_lh
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->stance_lh = tmp ? true : false;
  }

  // Field name: stance_rh
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->stance_rh = tmp ? true : false;
  }

  return true;
}  // NOLINT(readability/fn_size)


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_state_estimator_msgs
size_t get_serialized_size_state_estimator_msgs__msg__ContactDetection(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _ContactDetection__ros_msg_type * ros_message = static_cast<const _ContactDetection__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: header
  current_alignment += get_serialized_size_std_msgs__msg__Header(
    &(ros_message->header), current_alignment);

  // Field name: stance_lf
  {
    size_t item_size = sizeof(ros_message->stance_lf);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: stance_rf
  {
    size_t item_size = sizeof(ros_message->stance_rf);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: stance_lh
  {
    size_t item_size = sizeof(ros_message->stance_lh);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: stance_rh
  {
    size_t item_size = sizeof(ros_message->stance_rh);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_state_estimator_msgs
size_t max_serialized_size_state_estimator_msgs__msg__ContactDetection(
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

  // Field name: stance_lf
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: stance_rf
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: stance_lh
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: stance_rh
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }


  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = state_estimator_msgs__msg__ContactDetection;
    is_plain =
      (
      offsetof(DataType, stance_rh) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_state_estimator_msgs
bool cdr_serialize_key_state_estimator_msgs__msg__ContactDetection(
  const state_estimator_msgs__msg__ContactDetection * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: header
  {
    cdr_serialize_key_std_msgs__msg__Header(
      &ros_message->header, cdr);
  }

  // Field name: stance_lf
  {
    cdr << (ros_message->stance_lf ? true : false);
  }

  // Field name: stance_rf
  {
    cdr << (ros_message->stance_rf ? true : false);
  }

  // Field name: stance_lh
  {
    cdr << (ros_message->stance_lh ? true : false);
  }

  // Field name: stance_rh
  {
    cdr << (ros_message->stance_rh ? true : false);
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_state_estimator_msgs
size_t get_serialized_size_key_state_estimator_msgs__msg__ContactDetection(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _ContactDetection__ros_msg_type * ros_message = static_cast<const _ContactDetection__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;

  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: header
  current_alignment += get_serialized_size_key_std_msgs__msg__Header(
    &(ros_message->header), current_alignment);

  // Field name: stance_lf
  {
    size_t item_size = sizeof(ros_message->stance_lf);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: stance_rf
  {
    size_t item_size = sizeof(ros_message->stance_rf);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: stance_lh
  {
    size_t item_size = sizeof(ros_message->stance_lh);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: stance_rh
  {
    size_t item_size = sizeof(ros_message->stance_rh);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_state_estimator_msgs
size_t max_serialized_size_key_state_estimator_msgs__msg__ContactDetection(
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

  // Field name: stance_lf
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: stance_rf
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: stance_lh
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: stance_rh
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = state_estimator_msgs__msg__ContactDetection;
    is_plain =
      (
      offsetof(DataType, stance_rh) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}


static bool _ContactDetection__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const state_estimator_msgs__msg__ContactDetection * ros_message = static_cast<const state_estimator_msgs__msg__ContactDetection *>(untyped_ros_message);
  (void)ros_message;
  return cdr_serialize_state_estimator_msgs__msg__ContactDetection(ros_message, cdr);
}

static bool _ContactDetection__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  state_estimator_msgs__msg__ContactDetection * ros_message = static_cast<state_estimator_msgs__msg__ContactDetection *>(untyped_ros_message);
  (void)ros_message;
  return cdr_deserialize_state_estimator_msgs__msg__ContactDetection(cdr, ros_message);
}

static uint32_t _ContactDetection__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_state_estimator_msgs__msg__ContactDetection(
      untyped_ros_message, 0));
}

static size_t _ContactDetection__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_state_estimator_msgs__msg__ContactDetection(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_ContactDetection = {
  "state_estimator_msgs::msg",
  "ContactDetection",
  _ContactDetection__cdr_serialize,
  _ContactDetection__cdr_deserialize,
  _ContactDetection__get_serialized_size,
  _ContactDetection__max_serialized_size,
  nullptr
};

static rosidl_message_type_support_t _ContactDetection__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_ContactDetection,
  get_message_typesupport_handle_function,
  &state_estimator_msgs__msg__ContactDetection__get_type_hash,
  &state_estimator_msgs__msg__ContactDetection__get_type_description,
  &state_estimator_msgs__msg__ContactDetection__get_type_description_sources,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, state_estimator_msgs, msg, ContactDetection)() {
  return &_ContactDetection__type_support;
}

#if defined(__cplusplus)
}
#endif
