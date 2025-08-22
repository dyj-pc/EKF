// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__rosidl_typesupport_fastrtps_cpp.hpp.em
// with input from auto_aim:msg/SerialData.idl
// generated code does not contain a copyright notice

#ifndef AUTO_AIM__MSG__DETAIL__SERIAL_DATA__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
#define AUTO_AIM__MSG__DETAIL__SERIAL_DATA__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "auto_aim/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
#include "auto_aim/msg/detail/serial_data__struct.hpp"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

#include "fastcdr/Cdr.h"

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
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_auto_aim
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  auto_aim::msg::SerialData & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_auto_aim
get_serialized_size(
  const auto_aim::msg::SerialData & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_auto_aim
max_serialized_size_SerialData(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace auto_aim

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_auto_aim
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, auto_aim, msg, SerialData)();

#ifdef __cplusplus
}
#endif

#endif  // AUTO_AIM__MSG__DETAIL__SERIAL_DATA__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
