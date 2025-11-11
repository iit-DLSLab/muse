// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from state_estimator_msgs:msg/ContactDetection.idl
// generated code does not contain a copyright notice

#include "state_estimator_msgs/msg/detail/contact_detection__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_state_estimator_msgs
const rosidl_type_hash_t *
state_estimator_msgs__msg__ContactDetection__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x7f, 0x86, 0xfd, 0x7f, 0xbe, 0xc1, 0xf0, 0x15,
      0xcf, 0x90, 0x12, 0x60, 0xa1, 0xb5, 0x93, 0x29,
      0x3c, 0x94, 0xb7, 0x99, 0xff, 0x93, 0x3b, 0x79,
      0xc0, 0x7f, 0x17, 0xaf, 0x48, 0x9d, 0xe0, 0x83,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "builtin_interfaces/msg/detail/time__functions.h"
#include "std_msgs/msg/detail/header__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t builtin_interfaces__msg__Time__EXPECTED_HASH = {1, {
    0xb1, 0x06, 0x23, 0x5e, 0x25, 0xa4, 0xc5, 0xed,
    0x35, 0x09, 0x8a, 0xa0, 0xa6, 0x1a, 0x3e, 0xe9,
    0xc9, 0xb1, 0x8d, 0x19, 0x7f, 0x39, 0x8b, 0x0e,
    0x42, 0x06, 0xce, 0xa9, 0xac, 0xf9, 0xc1, 0x97,
  }};
static const rosidl_type_hash_t std_msgs__msg__Header__EXPECTED_HASH = {1, {
    0xf4, 0x9f, 0xb3, 0xae, 0x2c, 0xf0, 0x70, 0xf7,
    0x93, 0x64, 0x5f, 0xf7, 0x49, 0x68, 0x3a, 0xc6,
    0xb0, 0x62, 0x03, 0xe4, 0x1c, 0x89, 0x1e, 0x17,
    0x70, 0x1b, 0x1c, 0xb5, 0x97, 0xce, 0x6a, 0x01,
  }};
#endif

static char state_estimator_msgs__msg__ContactDetection__TYPE_NAME[] = "state_estimator_msgs/msg/ContactDetection";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char std_msgs__msg__Header__TYPE_NAME[] = "std_msgs/msg/Header";

// Define type names, field names, and default values
static char state_estimator_msgs__msg__ContactDetection__FIELD_NAME__header[] = "header";
static char state_estimator_msgs__msg__ContactDetection__FIELD_NAME__stance_lf[] = "stance_lf";
static char state_estimator_msgs__msg__ContactDetection__FIELD_NAME__stance_rf[] = "stance_rf";
static char state_estimator_msgs__msg__ContactDetection__FIELD_NAME__stance_lh[] = "stance_lh";
static char state_estimator_msgs__msg__ContactDetection__FIELD_NAME__stance_rh[] = "stance_rh";

static rosidl_runtime_c__type_description__Field state_estimator_msgs__msg__ContactDetection__FIELDS[] = {
  {
    {state_estimator_msgs__msg__ContactDetection__FIELD_NAME__header, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {std_msgs__msg__Header__TYPE_NAME, 19, 19},
    },
    {NULL, 0, 0},
  },
  {
    {state_estimator_msgs__msg__ContactDetection__FIELD_NAME__stance_lf, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {state_estimator_msgs__msg__ContactDetection__FIELD_NAME__stance_rf, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {state_estimator_msgs__msg__ContactDetection__FIELD_NAME__stance_lh, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {state_estimator_msgs__msg__ContactDetection__FIELD_NAME__stance_rh, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription state_estimator_msgs__msg__ContactDetection__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {std_msgs__msg__Header__TYPE_NAME, 19, 19},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
state_estimator_msgs__msg__ContactDetection__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {state_estimator_msgs__msg__ContactDetection__TYPE_NAME, 41, 41},
      {state_estimator_msgs__msg__ContactDetection__FIELDS, 5, 5},
    },
    {state_estimator_msgs__msg__ContactDetection__REFERENCED_TYPE_DESCRIPTIONS, 2, 2},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&std_msgs__msg__Header__EXPECTED_HASH, std_msgs__msg__Header__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = std_msgs__msg__Header__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "std_msgs/Header header\n"
  "\n"
  "bool stance_lf\n"
  "bool stance_rf\n"
  "bool stance_lh\n"
  "bool stance_rh";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
state_estimator_msgs__msg__ContactDetection__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {state_estimator_msgs__msg__ContactDetection__TYPE_NAME, 41, 41},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 83, 83},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
state_estimator_msgs__msg__ContactDetection__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[3];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 3, 3};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *state_estimator_msgs__msg__ContactDetection__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *std_msgs__msg__Header__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
