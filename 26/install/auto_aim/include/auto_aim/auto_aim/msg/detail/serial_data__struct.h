// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from auto_aim:msg/SerialData.idl
// generated code does not contain a copyright notice

#ifndef AUTO_AIM__MSG__DETAIL__SERIAL_DATA__STRUCT_H_
#define AUTO_AIM__MSG__DETAIL__SERIAL_DATA__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/SerialData in the package auto_aim.
typedef struct auto_aim__msg__SerialData
{
  /// 子弹速度
  float bullet_velocity;
  /// 子弹角度
  float bullet_angle;
  /// 云台当前偏航角
  int16_t gimbal_yaw;
  /// 敌方颜色(0:红色, 1:蓝色)
  uint8_t color;
} auto_aim__msg__SerialData;

// Struct for a sequence of auto_aim__msg__SerialData.
typedef struct auto_aim__msg__SerialData__Sequence
{
  auto_aim__msg__SerialData * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} auto_aim__msg__SerialData__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AUTO_AIM__MSG__DETAIL__SERIAL_DATA__STRUCT_H_
