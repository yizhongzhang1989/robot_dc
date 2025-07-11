// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from modbus_driver_interfaces:msg/MotorSimulationStatus.idl
// generated code does not contain a copyright notice

#ifndef MODBUS_DRIVER_INTERFACES__MSG__DETAIL__MOTOR_SIMULATION_STATUS__BUILDER_HPP_
#define MODBUS_DRIVER_INTERFACES__MSG__DETAIL__MOTOR_SIMULATION_STATUS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "modbus_driver_interfaces/msg/detail/motor_simulation_status__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace modbus_driver_interfaces
{

namespace msg
{

namespace builder
{

class Init_MotorSimulationStatus_last_command
{
public:
  explicit Init_MotorSimulationStatus_last_command(::modbus_driver_interfaces::msg::MotorSimulationStatus & msg)
  : msg_(msg)
  {}
  ::modbus_driver_interfaces::msg::MotorSimulationStatus last_command(::modbus_driver_interfaces::msg::MotorSimulationStatus::_last_command_type arg)
  {
    msg_.last_command = std::move(arg);
    return std::move(msg_);
  }

private:
  ::modbus_driver_interfaces::msg::MotorSimulationStatus msg_;
};

class Init_MotorSimulationStatus_moving
{
public:
  explicit Init_MotorSimulationStatus_moving(::modbus_driver_interfaces::msg::MotorSimulationStatus & msg)
  : msg_(msg)
  {}
  Init_MotorSimulationStatus_last_command moving(::modbus_driver_interfaces::msg::MotorSimulationStatus::_moving_type arg)
  {
    msg_.moving = std::move(arg);
    return Init_MotorSimulationStatus_last_command(msg_);
  }

private:
  ::modbus_driver_interfaces::msg::MotorSimulationStatus msg_;
};

class Init_MotorSimulationStatus_motion_mode
{
public:
  explicit Init_MotorSimulationStatus_motion_mode(::modbus_driver_interfaces::msg::MotorSimulationStatus & msg)
  : msg_(msg)
  {}
  Init_MotorSimulationStatus_moving motion_mode(::modbus_driver_interfaces::msg::MotorSimulationStatus::_motion_mode_type arg)
  {
    msg_.motion_mode = std::move(arg);
    return Init_MotorSimulationStatus_moving(msg_);
  }

private:
  ::modbus_driver_interfaces::msg::MotorSimulationStatus msg_;
};

class Init_MotorSimulationStatus_velocity
{
public:
  explicit Init_MotorSimulationStatus_velocity(::modbus_driver_interfaces::msg::MotorSimulationStatus & msg)
  : msg_(msg)
  {}
  Init_MotorSimulationStatus_motion_mode velocity(::modbus_driver_interfaces::msg::MotorSimulationStatus::_velocity_type arg)
  {
    msg_.velocity = std::move(arg);
    return Init_MotorSimulationStatus_motion_mode(msg_);
  }

private:
  ::modbus_driver_interfaces::msg::MotorSimulationStatus msg_;
};

class Init_MotorSimulationStatus_target_position
{
public:
  explicit Init_MotorSimulationStatus_target_position(::modbus_driver_interfaces::msg::MotorSimulationStatus & msg)
  : msg_(msg)
  {}
  Init_MotorSimulationStatus_velocity target_position(::modbus_driver_interfaces::msg::MotorSimulationStatus::_target_position_type arg)
  {
    msg_.target_position = std::move(arg);
    return Init_MotorSimulationStatus_velocity(msg_);
  }

private:
  ::modbus_driver_interfaces::msg::MotorSimulationStatus msg_;
};

class Init_MotorSimulationStatus_current_position
{
public:
  explicit Init_MotorSimulationStatus_current_position(::modbus_driver_interfaces::msg::MotorSimulationStatus & msg)
  : msg_(msg)
  {}
  Init_MotorSimulationStatus_target_position current_position(::modbus_driver_interfaces::msg::MotorSimulationStatus::_current_position_type arg)
  {
    msg_.current_position = std::move(arg);
    return Init_MotorSimulationStatus_target_position(msg_);
  }

private:
  ::modbus_driver_interfaces::msg::MotorSimulationStatus msg_;
};

class Init_MotorSimulationStatus_motor_id
{
public:
  Init_MotorSimulationStatus_motor_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MotorSimulationStatus_current_position motor_id(::modbus_driver_interfaces::msg::MotorSimulationStatus::_motor_id_type arg)
  {
    msg_.motor_id = std::move(arg);
    return Init_MotorSimulationStatus_current_position(msg_);
  }

private:
  ::modbus_driver_interfaces::msg::MotorSimulationStatus msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::modbus_driver_interfaces::msg::MotorSimulationStatus>()
{
  return modbus_driver_interfaces::msg::builder::Init_MotorSimulationStatus_motor_id();
}

}  // namespace modbus_driver_interfaces

#endif  // MODBUS_DRIVER_INTERFACES__MSG__DETAIL__MOTOR_SIMULATION_STATUS__BUILDER_HPP_
