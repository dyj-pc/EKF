// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from auto_aim:msg/SerialData.idl
// generated code does not contain a copyright notice

#ifndef AUTO_AIM__MSG__DETAIL__SERIAL_DATA__BUILDER_HPP_
#define AUTO_AIM__MSG__DETAIL__SERIAL_DATA__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "auto_aim/msg/detail/serial_data__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace auto_aim
{

namespace msg
{

namespace builder
{

class Init_SerialData_color
{
public:
  explicit Init_SerialData_color(::auto_aim::msg::SerialData & msg)
  : msg_(msg)
  {}
  ::auto_aim::msg::SerialData color(::auto_aim::msg::SerialData::_color_type arg)
  {
    msg_.color = std::move(arg);
    return std::move(msg_);
  }

private:
  ::auto_aim::msg::SerialData msg_;
};

class Init_SerialData_gimbal_yaw
{
public:
  explicit Init_SerialData_gimbal_yaw(::auto_aim::msg::SerialData & msg)
  : msg_(msg)
  {}
  Init_SerialData_color gimbal_yaw(::auto_aim::msg::SerialData::_gimbal_yaw_type arg)
  {
    msg_.gimbal_yaw = std::move(arg);
    return Init_SerialData_color(msg_);
  }

private:
  ::auto_aim::msg::SerialData msg_;
};

class Init_SerialData_bullet_angle
{
public:
  explicit Init_SerialData_bullet_angle(::auto_aim::msg::SerialData & msg)
  : msg_(msg)
  {}
  Init_SerialData_gimbal_yaw bullet_angle(::auto_aim::msg::SerialData::_bullet_angle_type arg)
  {
    msg_.bullet_angle = std::move(arg);
    return Init_SerialData_gimbal_yaw(msg_);
  }

private:
  ::auto_aim::msg::SerialData msg_;
};

class Init_SerialData_bullet_velocity
{
public:
  Init_SerialData_bullet_velocity()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SerialData_bullet_angle bullet_velocity(::auto_aim::msg::SerialData::_bullet_velocity_type arg)
  {
    msg_.bullet_velocity = std::move(arg);
    return Init_SerialData_bullet_angle(msg_);
  }

private:
  ::auto_aim::msg::SerialData msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::auto_aim::msg::SerialData>()
{
  return auto_aim::msg::builder::Init_SerialData_bullet_velocity();
}

}  // namespace auto_aim

#endif  // AUTO_AIM__MSG__DETAIL__SERIAL_DATA__BUILDER_HPP_
