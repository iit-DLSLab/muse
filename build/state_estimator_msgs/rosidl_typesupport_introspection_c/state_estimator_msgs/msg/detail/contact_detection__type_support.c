// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from state_estimator_msgs:msg/ContactDetection.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "state_estimator_msgs/msg/detail/contact_detection__rosidl_typesupport_introspection_c.h"
#include "state_estimator_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "state_estimator_msgs/msg/detail/contact_detection__functions.h"
#include "state_estimator_msgs/msg/detail/contact_detection__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void state_estimator_msgs__msg__ContactDetection__rosidl_typesupport_introspection_c__ContactDetection_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  state_estimator_msgs__msg__ContactDetection__init(message_memory);
}

void state_estimator_msgs__msg__ContactDetection__rosidl_typesupport_introspection_c__ContactDetection_fini_function(void * message_memory)
{
  state_estimator_msgs__msg__ContactDetection__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember state_estimator_msgs__msg__ContactDetection__rosidl_typesupport_introspection_c__ContactDetection_message_member_array[5] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(state_estimator_msgs__msg__ContactDetection, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "stance_lf",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(state_estimator_msgs__msg__ContactDetection, stance_lf),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "stance_rf",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(state_estimator_msgs__msg__ContactDetection, stance_rf),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "stance_lh",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(state_estimator_msgs__msg__ContactDetection, stance_lh),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "stance_rh",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(state_estimator_msgs__msg__ContactDetection, stance_rh),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers state_estimator_msgs__msg__ContactDetection__rosidl_typesupport_introspection_c__ContactDetection_message_members = {
  "state_estimator_msgs__msg",  // message namespace
  "ContactDetection",  // message name
  5,  // number of fields
  sizeof(state_estimator_msgs__msg__ContactDetection),
  false,  // has_any_key_member_
  state_estimator_msgs__msg__ContactDetection__rosidl_typesupport_introspection_c__ContactDetection_message_member_array,  // message members
  state_estimator_msgs__msg__ContactDetection__rosidl_typesupport_introspection_c__ContactDetection_init_function,  // function to initialize message memory (memory has to be allocated)
  state_estimator_msgs__msg__ContactDetection__rosidl_typesupport_introspection_c__ContactDetection_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t state_estimator_msgs__msg__ContactDetection__rosidl_typesupport_introspection_c__ContactDetection_message_type_support_handle = {
  0,
  &state_estimator_msgs__msg__ContactDetection__rosidl_typesupport_introspection_c__ContactDetection_message_members,
  get_message_typesupport_handle_function,
  &state_estimator_msgs__msg__ContactDetection__get_type_hash,
  &state_estimator_msgs__msg__ContactDetection__get_type_description,
  &state_estimator_msgs__msg__ContactDetection__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_state_estimator_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, state_estimator_msgs, msg, ContactDetection)() {
  state_estimator_msgs__msg__ContactDetection__rosidl_typesupport_introspection_c__ContactDetection_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  if (!state_estimator_msgs__msg__ContactDetection__rosidl_typesupport_introspection_c__ContactDetection_message_type_support_handle.typesupport_identifier) {
    state_estimator_msgs__msg__ContactDetection__rosidl_typesupport_introspection_c__ContactDetection_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &state_estimator_msgs__msg__ContactDetection__rosidl_typesupport_introspection_c__ContactDetection_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
