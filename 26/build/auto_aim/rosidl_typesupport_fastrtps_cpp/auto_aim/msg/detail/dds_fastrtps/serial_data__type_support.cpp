// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from auto_aim:msg/SerialData.idl
// generated code does not contain a copyright notice
#include "auto_aim/msg/detail/serial_data__rosidl_typesupport_fastrtps_cpp.hpp"
#include "auto_aim/msg/detail/serial_data__struct.hpp"

#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions

namespace auto_aim
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_auto_aim
cdr_serialize(
  const auto_aim::msg::SerialData & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: bullet_velocity
  cdr << ros_message.bullet_velocity;
  // Member: bullet_angle
  cdr << ros_message.bullet_angle;
  // Member: gimbal_yaw
  cdr << ros_message.gimbal_yaw;
  // Member: color
  cdr << ros_message.color;
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_auto_aim
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  auto_aim::msg::SerialData & ros_message)
{
  // Member: bullet_velocity
  cdr >> ros_message.bullet_velocity;

  // Member: bullet_angle
  cdr >> ros_message.bullet_angle;

  // Member: gimbal_yaw
  cdr >> ros_message.gimbal_yaw;

  // Member: color
  cdr >> ros_message.color;

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_auto_aim
get_serialized_size(
  const auto_aim::msg::SerialData & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: bullet_velocity
  {
    size_t item_size = sizeof(ros_message.bullet_velocity);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: bullet_angle
  {
    size_t item_size = sizeof(ros_message.bullet_angle);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: gimbal_yaw
  {
    size_t item_size = sizeof(ros_message.gimbal_yaw);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: color
  {
    size_t item_size = sizeof(ros_message.color);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_auto_aim
max_serialized_size_SerialData(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;


  // Member: bullet_velocity
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: bullet_angle
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: gimbal_yaw
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint16_t);
    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }

  // Member: color
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = auto_aim::msg::SerialData;
    is_plain =
      (
      offsetof(DataType, color) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _SerialData__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const auto_aim::msg::SerialData *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _SerialData__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<auto_aim::msg::SerialData *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _SerialData__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const auto_aim::msg::SerialData *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _SerialData__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_SerialData(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _SerialData__callbacks = {
  "auto_aim::msg",
  "SerialData",
  _SerialData__cdr_serialize,
  _SerialData__cdr_deserialize,
  _SerialData__get_serialized_size,
  _SerialData__max_serialized_size
};

static rosidl_message_type_support_t _SerialData__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_SerialData__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace auto_aim

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_auto_aim
const rosidl_message_type_support_t *
get_message_type_support_handle<auto_aim::msg::SerialData>()
{
  return &auto_aim::msg::typesupport_fastrtps_cpp::_SerialData__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, auto_aim, msg, SerialData)() {
  return &auto_aim::msg::typesupport_fastrtps_cpp::_SerialData__handle;
}

#ifdef __cplusplus
}
#endif
