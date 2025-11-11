// generated from rosidl_typesupport_c/resource/idl__type_support.cpp.em
// with input from state_estimator_msgs:msg/LegOdometry.idl
// generated code does not contain a copyright notice

#include "cstddef"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "state_estimator_msgs/msg/detail/leg_odometry__struct.h"
#include "state_estimator_msgs/msg/detail/leg_odometry__type_support.h"
#include "state_estimator_msgs/msg/detail/leg_odometry__functions.h"
#include "rosidl_typesupport_c/identifier.h"
#include "rosidl_typesupport_c/message_type_support_dispatch.h"
#include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_c/visibility_control.h"
#include "rosidl_typesupport_interface/macros.h"

namespace state_estimator_msgs
{

namespace msg
{

namespace rosidl_typesupport_c
{

typedef struct _LegOdometry_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _LegOdometry_type_support_ids_t;

static const _LegOdometry_type_support_ids_t _LegOdometry_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _LegOdometry_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _LegOdometry_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _LegOdometry_type_support_symbol_names_t _LegOdometry_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, state_estimator_msgs, msg, LegOdometry)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, state_estimator_msgs, msg, LegOdometry)),
  }
};

typedef struct _LegOdometry_type_support_data_t
{
  void * data[2];
} _LegOdometry_type_support_data_t;

static _LegOdometry_type_support_data_t _LegOdometry_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _LegOdometry_message_typesupport_map = {
  2,
  "state_estimator_msgs",
  &_LegOdometry_message_typesupport_ids.typesupport_identifier[0],
  &_LegOdometry_message_typesupport_symbol_names.symbol_name[0],
  &_LegOdometry_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t LegOdometry_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_LegOdometry_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
  &state_estimator_msgs__msg__LegOdometry__get_type_hash,
  &state_estimator_msgs__msg__LegOdometry__get_type_description,
  &state_estimator_msgs__msg__LegOdometry__get_type_description_sources,
};

}  // namespace rosidl_typesupport_c

}  // namespace msg

}  // namespace state_estimator_msgs

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, state_estimator_msgs, msg, LegOdometry)() {
  return &::state_estimator_msgs::msg::rosidl_typesupport_c::LegOdometry_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
