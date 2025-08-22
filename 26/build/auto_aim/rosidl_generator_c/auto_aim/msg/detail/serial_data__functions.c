// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from auto_aim:msg/SerialData.idl
// generated code does not contain a copyright notice
#include "auto_aim/msg/detail/serial_data__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
auto_aim__msg__SerialData__init(auto_aim__msg__SerialData * msg)
{
  if (!msg) {
    return false;
  }
  // bullet_velocity
  // bullet_angle
  // gimbal_yaw
  // color
  return true;
}

void
auto_aim__msg__SerialData__fini(auto_aim__msg__SerialData * msg)
{
  if (!msg) {
    return;
  }
  // bullet_velocity
  // bullet_angle
  // gimbal_yaw
  // color
}

bool
auto_aim__msg__SerialData__are_equal(const auto_aim__msg__SerialData * lhs, const auto_aim__msg__SerialData * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // bullet_velocity
  if (lhs->bullet_velocity != rhs->bullet_velocity) {
    return false;
  }
  // bullet_angle
  if (lhs->bullet_angle != rhs->bullet_angle) {
    return false;
  }
  // gimbal_yaw
  if (lhs->gimbal_yaw != rhs->gimbal_yaw) {
    return false;
  }
  // color
  if (lhs->color != rhs->color) {
    return false;
  }
  return true;
}

bool
auto_aim__msg__SerialData__copy(
  const auto_aim__msg__SerialData * input,
  auto_aim__msg__SerialData * output)
{
  if (!input || !output) {
    return false;
  }
  // bullet_velocity
  output->bullet_velocity = input->bullet_velocity;
  // bullet_angle
  output->bullet_angle = input->bullet_angle;
  // gimbal_yaw
  output->gimbal_yaw = input->gimbal_yaw;
  // color
  output->color = input->color;
  return true;
}

auto_aim__msg__SerialData *
auto_aim__msg__SerialData__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  auto_aim__msg__SerialData * msg = (auto_aim__msg__SerialData *)allocator.allocate(sizeof(auto_aim__msg__SerialData), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(auto_aim__msg__SerialData));
  bool success = auto_aim__msg__SerialData__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
auto_aim__msg__SerialData__destroy(auto_aim__msg__SerialData * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    auto_aim__msg__SerialData__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
auto_aim__msg__SerialData__Sequence__init(auto_aim__msg__SerialData__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  auto_aim__msg__SerialData * data = NULL;

  if (size) {
    data = (auto_aim__msg__SerialData *)allocator.zero_allocate(size, sizeof(auto_aim__msg__SerialData), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = auto_aim__msg__SerialData__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        auto_aim__msg__SerialData__fini(&data[i - 1]);
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
auto_aim__msg__SerialData__Sequence__fini(auto_aim__msg__SerialData__Sequence * array)
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
      auto_aim__msg__SerialData__fini(&array->data[i]);
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

auto_aim__msg__SerialData__Sequence *
auto_aim__msg__SerialData__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  auto_aim__msg__SerialData__Sequence * array = (auto_aim__msg__SerialData__Sequence *)allocator.allocate(sizeof(auto_aim__msg__SerialData__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = auto_aim__msg__SerialData__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
auto_aim__msg__SerialData__Sequence__destroy(auto_aim__msg__SerialData__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    auto_aim__msg__SerialData__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
auto_aim__msg__SerialData__Sequence__are_equal(const auto_aim__msg__SerialData__Sequence * lhs, const auto_aim__msg__SerialData__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!auto_aim__msg__SerialData__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
auto_aim__msg__SerialData__Sequence__copy(
  const auto_aim__msg__SerialData__Sequence * input,
  auto_aim__msg__SerialData__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(auto_aim__msg__SerialData);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    auto_aim__msg__SerialData * data =
      (auto_aim__msg__SerialData *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!auto_aim__msg__SerialData__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          auto_aim__msg__SerialData__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!auto_aim__msg__SerialData__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
