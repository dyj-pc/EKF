// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from auto_aim:msg/SerialData.idl
// generated code does not contain a copyright notice

#ifndef AUTO_AIM__MSG__DETAIL__SERIAL_DATA__TRAITS_HPP_
#define AUTO_AIM__MSG__DETAIL__SERIAL_DATA__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "auto_aim/msg/detail/serial_data__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace auto_aim
{

namespace msg
{

inline void to_flow_style_yaml(
  const SerialData & msg,
  std::ostream & out)
{
  out << "{";
  // member: bullet_velocity
  {
    out << "bullet_velocity: ";
    rosidl_generator_traits::value_to_yaml(msg.bullet_velocity, out);
    out << ", ";
  }

  // member: bullet_angle
  {
    out << "bullet_angle: ";
    rosidl_generator_traits::value_to_yaml(msg.bullet_angle, out);
    out << ", ";
  }

  // member: gimbal_yaw
  {
    out << "gimbal_yaw: ";
    rosidl_generator_traits::value_to_yaml(msg.gimbal_yaw, out);
    out << ", ";
  }

  // member: color
  {
    out << "color: ";
    rosidl_generator_traits::value_to_yaml(msg.color, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SerialData & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: bullet_velocity
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "bullet_velocity: ";
    rosidl_generator_traits::value_to_yaml(msg.bullet_velocity, out);
    out << "\n";
  }

  // member: bullet_angle
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "bullet_angle: ";
    rosidl_generator_traits::value_to_yaml(msg.bullet_angle, out);
    out << "\n";
  }

  // member: gimbal_yaw
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "gimbal_yaw: ";
    rosidl_generator_traits::value_to_yaml(msg.gimbal_yaw, out);
    out << "\n";
  }

  // member: color
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "color: ";
    rosidl_generator_traits::value_to_yaml(msg.color, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SerialData & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace auto_aim

namespace rosidl_generator_traits
{

[[deprecated("use auto_aim::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const auto_aim::msg::SerialData & msg,
  std::ostream & out, size_t indentation = 0)
{
  auto_aim::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use auto_aim::msg::to_yaml() instead")]]
inline std::string to_yaml(const auto_aim::msg::SerialData & msg)
{
  return auto_aim::msg::to_yaml(msg);
}

template<>
inline const char * data_type<auto_aim::msg::SerialData>()
{
  return "auto_aim::msg::SerialData";
}

template<>
inline const char * name<auto_aim::msg::SerialData>()
{
  return "auto_aim/msg/SerialData";
}

template<>
struct has_fixed_size<auto_aim::msg::SerialData>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<auto_aim::msg::SerialData>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<auto_aim::msg::SerialData>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // AUTO_AIM__MSG__DETAIL__SERIAL_DATA__TRAITS_HPP_
