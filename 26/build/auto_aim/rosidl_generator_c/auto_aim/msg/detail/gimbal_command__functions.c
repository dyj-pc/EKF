// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from auto_aim:msg/GimbalCommand.idl
// generated code does not contain a copyright notice
#include "auto_aim/msg/detail/gimbal_command__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
auto_aim__msg__GimbalCommand__init(auto_aim__msg__GimbalCommand * msg)
{
  if (!msg) {
    return false;
  }
  // pitch
  // yaw
  return true;
}

void
auto_aim__msg__GimbalCommand__fini(auto_aim__msg__GimbalCommand * msg)
{
  if (!msg) {
    return;
  }
  // pitch
  // yaw
}

bool
auto_aim__msg__GimbalCommand__are_equal(const auto_aim__msg__GimbalCommand * lhs, const auto_aim__msg__GimbalCommand * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // pitch
  if (lhs->pitch != rhs->pitch) {
    return false;
  }
  // yaw
  if (lhs->yaw != rhs->yaw) {
    return false;
  }
  return true;
}

bool
auto_aim__msg__GimbalCommand__copy(
  const auto_aim__msg__GimbalCommand * input,
  auto_aim__msg__GimbalCommand * output)
{
  if (!input || !output) {
    return false;
  }
  // pitch
  output->pitch = input->pitch;
  // yaw
  output->yaw = input->yaw;
  return true;
}

auto_aim__msg__GimbalCommand *
auto_aim__msg__GimbalCommand__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  auto_aim__msg__GimbalCommand * msg = (auto_aim__msg__GimbalCommand *)allocator.allocate(sizeof(auto_aim__msg__GimbalCommand), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(auto_aim__msg__GimbalCommand));
  bool success = auto_aim__msg__GimbalCommand__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
auto_aim__msg__GimbalCommand__destroy(auto_aim__msg__GimbalCommand * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    auto_aim__msg__GimbalCommand__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
auto_aim__msg__GimbalCommand__Sequence__init(auto_aim__msg__GimbalCommand__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  auto_aim__msg__GimbalCommand * data = NULL;

  if (size) {
    data = (auto_aim__msg__GimbalCommand *)allocator.zero_allocate(size, sizeof(auto_aim__msg__GimbalCommand), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = auto_aim__msg__GimbalCommand__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        auto_aim__msg__GimbalCommand__fini(&data[i - 1]);
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
auto_aim__msg__GimbalCommand__Sequence__fini(auto_aim__msg__GimbalCommand__Sequence * array)
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
      auto_aim__msg__GimbalCommand__fini(&array->data[i]);
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

auto_aim__msg__GimbalCommand__Sequence *
auto_aim__msg__GimbalCommand__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  auto_aim__msg__GimbalCommand__Sequence * array = (auto_aim__msg__GimbalCommand__Sequence *)allocator.allocate(sizeof(auto_aim__msg__GimbalCommand__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = auto_aim__msg__GimbalCommand__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
auto_aim__msg__GimbalCommand__Sequence__destroy(auto_aim__msg__GimbalCommand__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    auto_aim__msg__GimbalCommand__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
auto_aim__msg__GimbalCommand__Sequence__are_equal(const auto_aim__msg__GimbalCommand__Sequence * lhs, const auto_aim__msg__GimbalCommand__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!auto_aim__msg__GimbalCommand__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
auto_aim__msg__GimbalCommand__Sequence__copy(
  const auto_aim__msg__GimbalCommand__Sequence * input,
  auto_aim__msg__GimbalCommand__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(auto_aim__msg__GimbalCommand);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    auto_aim__msg__GimbalCommand * data =
      (auto_aim__msg__GimbalCommand *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!auto_aim__msg__GimbalCommand__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          auto_aim__msg__GimbalCommand__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!auto_aim__msg__GimbalCommand__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
