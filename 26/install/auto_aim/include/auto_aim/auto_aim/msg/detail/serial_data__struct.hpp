// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from auto_aim:msg/SerialData.idl
// generated code does not contain a copyright notice

#ifndef AUTO_AIM__MSG__DETAIL__SERIAL_DATA__STRUCT_HPP_
#define AUTO_AIM__MSG__DETAIL__SERIAL_DATA__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__auto_aim__msg__SerialData __attribute__((deprecated))
#else
# define DEPRECATED__auto_aim__msg__SerialData __declspec(deprecated)
#endif

namespace auto_aim
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct SerialData_
{
  using Type = SerialData_<ContainerAllocator>;

  explicit SerialData_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->bullet_velocity = 0.0f;
      this->bullet_angle = 0.0f;
      this->gimbal_yaw = 0;
      this->color = 0;
    }
  }

  explicit SerialData_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->bullet_velocity = 0.0f;
      this->bullet_angle = 0.0f;
      this->gimbal_yaw = 0;
      this->color = 0;
    }
  }

  // field types and members
  using _bullet_velocity_type =
    float;
  _bullet_velocity_type bullet_velocity;
  using _bullet_angle_type =
    float;
  _bullet_angle_type bullet_angle;
  using _gimbal_yaw_type =
    int16_t;
  _gimbal_yaw_type gimbal_yaw;
  using _color_type =
    uint8_t;
  _color_type color;

  // setters for named parameter idiom
  Type & set__bullet_velocity(
    const float & _arg)
  {
    this->bullet_velocity = _arg;
    return *this;
  }
  Type & set__bullet_angle(
    const float & _arg)
  {
    this->bullet_angle = _arg;
    return *this;
  }
  Type & set__gimbal_yaw(
    const int16_t & _arg)
  {
    this->gimbal_yaw = _arg;
    return *this;
  }
  Type & set__color(
    const uint8_t & _arg)
  {
    this->color = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    auto_aim::msg::SerialData_<ContainerAllocator> *;
  using ConstRawPtr =
    const auto_aim::msg::SerialData_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<auto_aim::msg::SerialData_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<auto_aim::msg::SerialData_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      auto_aim::msg::SerialData_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<auto_aim::msg::SerialData_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      auto_aim::msg::SerialData_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<auto_aim::msg::SerialData_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<auto_aim::msg::SerialData_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<auto_aim::msg::SerialData_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__auto_aim__msg__SerialData
    std::shared_ptr<auto_aim::msg::SerialData_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__auto_aim__msg__SerialData
    std::shared_ptr<auto_aim::msg::SerialData_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SerialData_ & other) const
  {
    if (this->bullet_velocity != other.bullet_velocity) {
      return false;
    }
    if (this->bullet_angle != other.bullet_angle) {
      return false;
    }
    if (this->gimbal_yaw != other.gimbal_yaw) {
      return false;
    }
    if (this->color != other.color) {
      return false;
    }
    return true;
  }
  bool operator!=(const SerialData_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SerialData_

// alias to use template instance with default allocator
using SerialData =
  auto_aim::msg::SerialData_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace auto_aim

#endif  // AUTO_AIM__MSG__DETAIL__SERIAL_DATA__STRUCT_HPP_
