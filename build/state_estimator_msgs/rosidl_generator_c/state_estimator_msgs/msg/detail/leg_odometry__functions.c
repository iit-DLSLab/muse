// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from state_estimator_msgs:msg/LegOdometry.idl
// generated code does not contain a copyright notice
#include "state_estimator_msgs/msg/detail/leg_odometry__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"

bool
state_estimator_msgs__msg__LegOdometry__init(state_estimator_msgs__msg__LegOdometry * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    state_estimator_msgs__msg__LegOdometry__fini(msg);
    return false;
  }
  // lin_vel_lf
  // lin_vel_rf
  // lin_vel_lh
  // lin_vel_rh
  // base_velocity
  return true;
}

void
state_estimator_msgs__msg__LegOdometry__fini(state_estimator_msgs__msg__LegOdometry * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // lin_vel_lf
  // lin_vel_rf
  // lin_vel_lh
  // lin_vel_rh
  // base_velocity
}

bool
state_estimator_msgs__msg__LegOdometry__are_equal(const state_estimator_msgs__msg__LegOdometry * lhs, const state_estimator_msgs__msg__LegOdometry * rhs)
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
  // lin_vel_lf
  for (size_t i = 0; i < 3; ++i) {
    if (lhs->lin_vel_lf[i] != rhs->lin_vel_lf[i]) {
      return false;
    }
  }
  // lin_vel_rf
  for (size_t i = 0; i < 3; ++i) {
    if (lhs->lin_vel_rf[i] != rhs->lin_vel_rf[i]) {
      return false;
    }
  }
  // lin_vel_lh
  for (size_t i = 0; i < 3; ++i) {
    if (lhs->lin_vel_lh[i] != rhs->lin_vel_lh[i]) {
      return false;
    }
  }
  // lin_vel_rh
  for (size_t i = 0; i < 3; ++i) {
    if (lhs->lin_vel_rh[i] != rhs->lin_vel_rh[i]) {
      return false;
    }
  }
  // base_velocity
  for (size_t i = 0; i < 3; ++i) {
    if (lhs->base_velocity[i] != rhs->base_velocity[i]) {
      return false;
    }
  }
  return true;
}

bool
state_estimator_msgs__msg__LegOdometry__copy(
  const state_estimator_msgs__msg__LegOdometry * input,
  state_estimator_msgs__msg__LegOdometry * output)
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
  // lin_vel_lf
  for (size_t i = 0; i < 3; ++i) {
    output->lin_vel_lf[i] = input->lin_vel_lf[i];
  }
  // lin_vel_rf
  for (size_t i = 0; i < 3; ++i) {
    output->lin_vel_rf[i] = input->lin_vel_rf[i];
  }
  // lin_vel_lh
  for (size_t i = 0; i < 3; ++i) {
    output->lin_vel_lh[i] = input->lin_vel_lh[i];
  }
  // lin_vel_rh
  for (size_t i = 0; i < 3; ++i) {
    output->lin_vel_rh[i] = input->lin_vel_rh[i];
  }
  // base_velocity
  for (size_t i = 0; i < 3; ++i) {
    output->base_velocity[i] = input->base_velocity[i];
  }
  return true;
}

state_estimator_msgs__msg__LegOdometry *
state_estimator_msgs__msg__LegOdometry__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  state_estimator_msgs__msg__LegOdometry * msg = (state_estimator_msgs__msg__LegOdometry *)allocator.allocate(sizeof(state_estimator_msgs__msg__LegOdometry), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(state_estimator_msgs__msg__LegOdometry));
  bool success = state_estimator_msgs__msg__LegOdometry__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
state_estimator_msgs__msg__LegOdometry__destroy(state_estimator_msgs__msg__LegOdometry * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    state_estimator_msgs__msg__LegOdometry__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
state_estimator_msgs__msg__LegOdometry__Sequence__init(state_estimator_msgs__msg__LegOdometry__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  state_estimator_msgs__msg__LegOdometry * data = NULL;

  if (size) {
    data = (state_estimator_msgs__msg__LegOdometry *)allocator.zero_allocate(size, sizeof(state_estimator_msgs__msg__LegOdometry), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = state_estimator_msgs__msg__LegOdometry__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        state_estimator_msgs__msg__LegOdometry__fini(&data[i - 1]);
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
state_estimator_msgs__msg__LegOdometry__Sequence__fini(state_estimator_msgs__msg__LegOdometry__Sequence * array)
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
      state_estimator_msgs__msg__LegOdometry__fini(&array->data[i]);
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

state_estimator_msgs__msg__LegOdometry__Sequence *
state_estimator_msgs__msg__LegOdometry__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  state_estimator_msgs__msg__LegOdometry__Sequence * array = (state_estimator_msgs__msg__LegOdometry__Sequence *)allocator.allocate(sizeof(state_estimator_msgs__msg__LegOdometry__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = state_estimator_msgs__msg__LegOdometry__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
state_estimator_msgs__msg__LegOdometry__Sequence__destroy(state_estimator_msgs__msg__LegOdometry__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    state_estimator_msgs__msg__LegOdometry__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
state_estimator_msgs__msg__LegOdometry__Sequence__are_equal(const state_estimator_msgs__msg__LegOdometry__Sequence * lhs, const state_estimator_msgs__msg__LegOdometry__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!state_estimator_msgs__msg__LegOdometry__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
state_estimator_msgs__msg__LegOdometry__Sequence__copy(
  const state_estimator_msgs__msg__LegOdometry__Sequence * input,
  state_estimator_msgs__msg__LegOdometry__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(state_estimator_msgs__msg__LegOdometry);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    state_estimator_msgs__msg__LegOdometry * data =
      (state_estimator_msgs__msg__LegOdometry *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!state_estimator_msgs__msg__LegOdometry__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          state_estimator_msgs__msg__LegOdometry__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!state_estimator_msgs__msg__LegOdometry__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
