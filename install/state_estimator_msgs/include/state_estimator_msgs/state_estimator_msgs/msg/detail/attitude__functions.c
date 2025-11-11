// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from state_estimator_msgs:msg/Attitude.idl
// generated code does not contain a copyright notice
#include "state_estimator_msgs/msg/detail/attitude__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"

bool
state_estimator_msgs__msg__Attitude__init(state_estimator_msgs__msg__Attitude * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    state_estimator_msgs__msg__Attitude__fini(msg);
    return false;
  }
  // quaternion
  // roll_deg
  // pitch_deg
  // yaw_deg
  // angular_velocity
  return true;
}

void
state_estimator_msgs__msg__Attitude__fini(state_estimator_msgs__msg__Attitude * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // quaternion
  // roll_deg
  // pitch_deg
  // yaw_deg
  // angular_velocity
}

bool
state_estimator_msgs__msg__Attitude__are_equal(const state_estimator_msgs__msg__Attitude * lhs, const state_estimator_msgs__msg__Attitude * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // quaternion
  for (size_t i = 0; i < 4; ++i) {
    if (lhs->quaternion[i] != rhs->quaternion[i]) {
      return false;
    }
  }
  // roll_deg
  if (lhs->roll_deg != rhs->roll_deg) {
    return false;
  }
  // pitch_deg
  if (lhs->pitch_deg != rhs->pitch_deg) {
    return false;
  }
  // yaw_deg
  if (lhs->yaw_deg != rhs->yaw_deg) {
    return false;
  }
  // angular_velocity
  for (size_t i = 0; i < 3; ++i) {
    if (lhs->angular_velocity[i] != rhs->angular_velocity[i]) {
      return false;
    }
  }
  return true;
}

bool
state_estimator_msgs__msg__Attitude__copy(
  const state_estimator_msgs__msg__Attitude * input,
  state_estimator_msgs__msg__Attitude * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // quaternion
  for (size_t i = 0; i < 4; ++i) {
    output->quaternion[i] = input->quaternion[i];
  }
  // roll_deg
  output->roll_deg = input->roll_deg;
  // pitch_deg
  output->pitch_deg = input->pitch_deg;
  // yaw_deg
  output->yaw_deg = input->yaw_deg;
  // angular_velocity
  for (size_t i = 0; i < 3; ++i) {
    output->angular_velocity[i] = input->angular_velocity[i];
  }
  return true;
}

state_estimator_msgs__msg__Attitude *
state_estimator_msgs__msg__Attitude__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  state_estimator_msgs__msg__Attitude * msg = (state_estimator_msgs__msg__Attitude *)allocator.allocate(sizeof(state_estimator_msgs__msg__Attitude), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(state_estimator_msgs__msg__Attitude));
  bool success = state_estimator_msgs__msg__Attitude__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
state_estimator_msgs__msg__Attitude__destroy(state_estimator_msgs__msg__Attitude * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    state_estimator_msgs__msg__Attitude__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
state_estimator_msgs__msg__Attitude__Sequence__init(state_estimator_msgs__msg__Attitude__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  state_estimator_msgs__msg__Attitude * data = NULL;

  if (size) {
    data = (state_estimator_msgs__msg__Attitude *)allocator.zero_allocate(size, sizeof(state_estimator_msgs__msg__Attitude), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = state_estimator_msgs__msg__Attitude__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        state_estimator_msgs__msg__Attitude__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
state_estimator_msgs__msg__Attitude__Sequence__fini(state_estimator_msgs__msg__Attitude__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      state_estimator_msgs__msg__Attitude__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

state_estimator_msgs__msg__Attitude__Sequence *
state_estimator_msgs__msg__Attitude__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  state_estimator_msgs__msg__Attitude__Sequence * array = (state_estimator_msgs__msg__Attitude__Sequence *)allocator.allocate(sizeof(state_estimator_msgs__msg__Attitude__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = state_estimator_msgs__msg__Attitude__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
state_estimator_msgs__msg__Attitude__Sequence__destroy(state_estimator_msgs__msg__Attitude__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    state_estimator_msgs__msg__Attitude__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
state_estimator_msgs__msg__Attitude__Sequence__are_equal(const state_estimator_msgs__msg__Attitude__Sequence * lhs, const state_estimator_msgs__msg__Attitude__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!state_estimator_msgs__msg__Attitude__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
state_estimator_msgs__msg__Attitude__Sequence__copy(
  const state_estimator_msgs__msg__Attitude__Sequence * input,
  state_estimator_msgs__msg__Attitude__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(state_estimator_msgs__msg__Attitude);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    state_estimator_msgs__msg__Attitude * data =
      (state_estimator_msgs__msg__Attitude *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!state_estimator_msgs__msg__Attitude__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          state_estimator_msgs__msg__Attitude__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!state_estimator_msgs__msg__Attitude__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
