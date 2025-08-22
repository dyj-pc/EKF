// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from auto_aim:msg/GimbalCommand.idl
// generated code does not contain a copyright notice

#ifndef AUTO_AIM__MSG__DETAIL__GIMBAL_COMMAND__STRUCT_H_
#define AUTO_AIM__MSG__DETAIL__GIMBAL_COMMAND__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/GimbalCommand in the package auto_aim.
typedef struct auto_aim__msg__GimbalCommand
{
  /// 俯仰角度命令
  float pitch;
  /// 偏航角度命令
  float yaw;
} auto_aim__msg__GimbalCommand;

// Struct for a sequence of auto_aim__msg__GimbalCommand.
typedef struct auto_aim__msg__GimbalCommand__Sequence
{
  auto_aim__msg__GimbalCommand * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} auto_aim__msg__GimbalCommand__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AUTO_AIM__MSG__DETAIL__GIMBAL_COMMAND__STRUCT_H_
