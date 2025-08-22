// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from auto_aim:msg/GimbalCommand.idl
// generated code does not contain a copyright notice

#ifndef AUTO_AIM__MSG__DETAIL__GIMBAL_COMMAND__BUILDER_HPP_
#define AUTO_AIM__MSG__DETAIL__GIMBAL_COMMAND__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "auto_aim/msg/detail/gimbal_command__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace auto_aim
{

namespace msg
{

namespace builder
{

class Init_GimbalCommand_yaw
{
public:
  explicit Init_GimbalCommand_yaw(::auto_aim::msg::GimbalCommand & msg)
  : msg_(msg)
  {}
  ::auto_aim::msg::GimbalCommand yaw(::auto_aim::msg::GimbalCommand::_yaw_type arg)
  {
    msg_.yaw = std::move(arg);
    return std::move(msg_);
  }

private:
  ::auto_aim::msg::GimbalCommand msg_;
};

class Init_GimbalCommand_pitch
{
public:
  Init_GimbalCommand_pitch()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GimbalCommand_yaw pitch(::auto_aim::msg::GimbalCommand::_pitch_type arg)
  {
    msg_.pitch = std::move(arg);
    return Init_GimbalCommand_yaw(msg_);
  }

private:
  ::auto_aim::msg::GimbalCommand msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::auto_aim::msg::GimbalCommand>()
{
  return auto_aim::msg::builder::Init_GimbalCommand_pitch();
}

}  // namespace auto_aim

#endif  // AUTO_AIM__MSG__DETAIL__GIMBAL_COMMAND__BUILDER_HPP_
