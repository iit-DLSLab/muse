// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from state_estimator_msgs:msg/SensorFusion.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "state_estimator_msgs/msg/sensor_fusion.h"


#ifndef STATE_ESTIMATOR_MSGS__MSG__DETAIL__SENSOR_FUSION__FUNCTIONS_H_
#define STATE_ESTIMATOR_MSGS__MSG__DETAIL__SENSOR_FUSION__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/action_type_support_struct.h"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_runtime_c/service_type_support_struct.h"
#include "rosidl_runtime_c/type_description/type_description__struct.h"
#include "rosidl_runtime_c/type_description/type_source__struct.h"
#include "rosidl_runtime_c/type_hash.h"
#include "rosidl_runtime_c/visibility_control.h"
#include "state_estimator_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "state_estimator_msgs/msg/detail/sensor_fusion__struct.h"

/// Initialize msg/SensorFusion message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * state_estimator_msgs__msg__SensorFusion
 * )) before or use
 * state_estimator_msgs__msg__SensorFusion__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_state_estimator_msgs
bool
state_estimator_msgs__msg__SensorFusion__init(state_estimator_msgs__msg__SensorFusion * msg);

/// Finalize msg/SensorFusion message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_state_estimator_msgs
void
state_estimator_msgs__msg__SensorFusion__fini(state_estimator_msgs__msg__SensorFusion * msg);

/// Create msg/SensorFusion message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * state_estimator_msgs__msg__SensorFusion__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_state_estimator_msgs
state_estimator_msgs__msg__SensorFusion *
state_estimator_msgs__msg__SensorFusion__create(void);

/// Destroy msg/SensorFusion message.
/**
 * It calls
 * state_estimator_msgs__msg__SensorFusion__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_state_estimator_msgs
void
state_estimator_msgs__msg__SensorFusion__destroy(state_estimator_msgs__msg__SensorFusion * msg);

/// Check for msg/SensorFusion message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_state_estimator_msgs
bool
state_estimator_msgs__msg__SensorFusion__are_equal(const state_estimator_msgs__msg__SensorFusion * lhs, const state_estimator_msgs__msg__SensorFusion * rhs);

/// Copy a msg/SensorFusion message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_state_estimator_msgs
bool
state_estimator_msgs__msg__SensorFusion__copy(
  const state_estimator_msgs__msg__SensorFusion * input,
  state_estimator_msgs__msg__SensorFusion * output);

/// Retrieve pointer to the hash of the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_state_estimator_msgs
const rosidl_type_hash_t *
state_estimator_msgs__msg__SensorFusion__get_type_hash(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_state_estimator_msgs
const rosidl_runtime_c__type_description__TypeDescription *
state_estimator_msgs__msg__SensorFusion__get_type_description(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the single raw source text that defined this type.
ROSIDL_GENERATOR_C_PUBLIC_state_estimator_msgs
const rosidl_runtime_c__type_description__TypeSource *
state_estimator_msgs__msg__SensorFusion__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the recursive raw sources that defined the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_state_estimator_msgs
const rosidl_runtime_c__type_description__TypeSource__Sequence *
state_estimator_msgs__msg__SensorFusion__get_type_description_sources(
  const rosidl_message_type_support_t * type_support);

/// Initialize array of msg/SensorFusion messages.
/**
 * It allocates the memory for the number of elements and calls
 * state_estimator_msgs__msg__SensorFusion__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_state_estimator_msgs
bool
state_estimator_msgs__msg__SensorFusion__Sequence__init(state_estimator_msgs__msg__SensorFusion__Sequence * array, size_t size);

/// Finalize array of msg/SensorFusion messages.
/**
 * It calls
 * state_estimator_msgs__msg__SensorFusion__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_state_estimator_msgs
void
state_estimator_msgs__msg__SensorFusion__Sequence__fini(state_estimator_msgs__msg__SensorFusion__Sequence * array);

/// Create array of msg/SensorFusion messages.
/**
 * It allocates the memory for the array and calls
 * state_estimator_msgs__msg__SensorFusion__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_state_estimator_msgs
state_estimator_msgs__msg__SensorFusion__Sequence *
state_estimator_msgs__msg__SensorFusion__Sequence__create(size_t size);

/// Destroy array of msg/SensorFusion messages.
/**
 * It calls
 * state_estimator_msgs__msg__SensorFusion__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_state_estimator_msgs
void
state_estimator_msgs__msg__SensorFusion__Sequence__destroy(state_estimator_msgs__msg__SensorFusion__Sequence * array);

/// Check for msg/SensorFusion message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_state_estimator_msgs
bool
state_estimator_msgs__msg__SensorFusion__Sequence__are_equal(const state_estimator_msgs__msg__SensorFusion__Sequence * lhs, const state_estimator_msgs__msg__SensorFusion__Sequence * rhs);

/// Copy an array of msg/SensorFusion messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_state_estimator_msgs
bool
state_estimator_msgs__msg__SensorFusion__Sequence__copy(
  const state_estimator_msgs__msg__SensorFusion__Sequence * input,
  state_estimator_msgs__msg__SensorFusion__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // STATE_ESTIMATOR_MSGS__MSG__DETAIL__SENSOR_FUSION__FUNCTIONS_H_
