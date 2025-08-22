// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from auto_aim:msg/GimbalCommand.idl
// generated code does not contain a copyright notice

#ifndef AUTO_AIM__MSG__DETAIL__GIMBAL_COMMAND__TRAITS_HPP_
#define AUTO_AIM__MSG__DETAIL__GIMBAL_COMMAND__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "auto_aim/msg/detail/gimbal_command__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace auto_aim
{

namespace msg
{

inline void to_flow_style_yaml(
  const GimbalCommand & msg,
  std::ostream & out)
{
  out << "{";
  // member: pitch
  {
    out << "pitch: ";
    rosidl_generator_traits::value_to_yaml(msg.pitch, out);
    out << ", ";
  }

  // member: yaw
  {
    out << "yaw: ";
    rosidl_generator_traits::value_to_yaml(msg.yaw, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GimbalCommand & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: pitch
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pitch: ";
    rosidl_generator_traits::value_to_yaml(msg.pitch, out);
    out << "\n";
  }

  // member: yaw
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "yaw: ";
    rosidl_generator_traits::value_to_yaml(msg.yaw, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GimbalCommand & msg, bool use_flow_style = false)
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
  const auto_aim::msg::GimbalCommand & msg,
  std::ostream & out, size_t indentation = 0)
{
  auto_aim::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use auto_aim::msg::to_yaml() instead")]]
inline std::string to_yaml(const auto_aim::msg::GimbalCommand & msg)
{
  return auto_aim::msg::to_yaml(msg);
}

template<>
inline const char * data_type<auto_aim::msg::GimbalCommand>()
{
  return "auto_aim::msg::GimbalCommand";
}

template<>
inline const char * name<auto_aim::msg::GimbalCommand>()
{
  return "auto_aim/msg/GimbalCommand";
}

template<>
struct has_fixed_size<auto_aim::msg::GimbalCommand>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<auto_aim::msg::GimbalCommand>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<auto_aim::msg::GimbalCommand>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // AUTO_AIM__MSG__DETAIL__GIMBAL_COMMAND__TRAITS_HPP_
