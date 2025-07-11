// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from modbus_driver_interfaces:msg/MotorSimulationStatus.idl
// generated code does not contain a copyright notice

#ifndef MODBUS_DRIVER_INTERFACES__MSG__DETAIL__MOTOR_SIMULATION_STATUS__STRUCT_HPP_
#define MODBUS_DRIVER_INTERFACES__MSG__DETAIL__MOTOR_SIMULATION_STATUS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__modbus_driver_interfaces__msg__MotorSimulationStatus __attribute__((deprecated))
#else
# define DEPRECATED__modbus_driver_interfaces__msg__MotorSimulationStatus __declspec(deprecated)
#endif

namespace modbus_driver_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MotorSimulationStatus_
{
  using Type = MotorSimulationStatus_<ContainerAllocator>;

  explicit MotorSimulationStatus_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->motor_id = 0l;
      this->current_position = 0.0f;
      this->target_position = 0.0f;
      this->velocity = 0.0f;
      this->motion_mode = 0l;
      this->moving = false;
      this->last_command = "";
    }
  }

  explicit MotorSimulationStatus_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : last_command(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->motor_id = 0l;
      this->current_position = 0.0f;
      this->target_position = 0.0f;
      this->velocity = 0.0f;
      this->motion_mode = 0l;
      this->moving = false;
      this->last_command = "";
    }
  }

  // field types and members
  using _motor_id_type =
    int32_t;
  _motor_id_type motor_id;
  using _current_position_type =
    float;
  _current_position_type current_position;
  using _target_position_type =
    float;
  _target_position_type target_position;
  using _velocity_type =
    float;
  _velocity_type velocity;
  using _motion_mode_type =
    int32_t;
  _motion_mode_type motion_mode;
  using _moving_type =
    bool;
  _moving_type moving;
  using _last_command_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _last_command_type last_command;

  // setters for named parameter idiom
  Type & set__motor_id(
    const int32_t & _arg)
  {
    this->motor_id = _arg;
    return *this;
  }
  Type & set__current_position(
    const float & _arg)
  {
    this->current_position = _arg;
    return *this;
  }
  Type & set__target_position(
    const float & _arg)
  {
    this->target_position = _arg;
    return *this;
  }
  Type & set__velocity(
    const float & _arg)
  {
    this->velocity = _arg;
    return *this;
  }
  Type & set__motion_mode(
    const int32_t & _arg)
  {
    this->motion_mode = _arg;
    return *this;
  }
  Type & set__moving(
    const bool & _arg)
  {
    this->moving = _arg;
    return *this;
  }
  Type & set__last_command(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->last_command = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    modbus_driver_interfaces::msg::MotorSimulationStatus_<ContainerAllocator> *;
  using ConstRawPtr =
    const modbus_driver_interfaces::msg::MotorSimulationStatus_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<modbus_driver_interfaces::msg::MotorSimulationStatus_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<modbus_driver_interfaces::msg::MotorSimulationStatus_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      modbus_driver_interfaces::msg::MotorSimulationStatus_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<modbus_driver_interfaces::msg::MotorSimulationStatus_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      modbus_driver_interfaces::msg::MotorSimulationStatus_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<modbus_driver_interfaces::msg::MotorSimulationStatus_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<modbus_driver_interfaces::msg::MotorSimulationStatus_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<modbus_driver_interfaces::msg::MotorSimulationStatus_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__modbus_driver_interfaces__msg__MotorSimulationStatus
    std::shared_ptr<modbus_driver_interfaces::msg::MotorSimulationStatus_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__modbus_driver_interfaces__msg__MotorSimulationStatus
    std::shared_ptr<modbus_driver_interfaces::msg::MotorSimulationStatus_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MotorSimulationStatus_ & other) const
  {
    if (this->motor_id != other.motor_id) {
      return false;
    }
    if (this->current_position != other.current_position) {
      return false;
    }
    if (this->target_position != other.target_position) {
      return false;
    }
    if (this->velocity != other.velocity) {
      return false;
    }
    if (this->motion_mode != other.motion_mode) {
      return false;
    }
    if (this->moving != other.moving) {
      return false;
    }
    if (this->last_command != other.last_command) {
      return false;
    }
    return true;
  }
  bool operator!=(const MotorSimulationStatus_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MotorSimulationStatus_

// alias to use template instance with default allocator
using MotorSimulationStatus =
  modbus_driver_interfaces::msg::MotorSimulationStatus_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace modbus_driver_interfaces

#endif  // MODBUS_DRIVER_INTERFACES__MSG__DETAIL__MOTOR_SIMULATION_STATUS__STRUCT_HPP_
