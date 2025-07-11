// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from modbus_driver_interfaces:msg/MotorSimulationStatus.idl
// generated code does not contain a copyright notice

#ifndef MODBUS_DRIVER_INTERFACES__MSG__DETAIL__MOTOR_SIMULATION_STATUS__TRAITS_HPP_
#define MODBUS_DRIVER_INTERFACES__MSG__DETAIL__MOTOR_SIMULATION_STATUS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "modbus_driver_interfaces/msg/detail/motor_simulation_status__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace modbus_driver_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const MotorSimulationStatus & msg,
  std::ostream & out)
{
  out << "{";
  // member: motor_id
  {
    out << "motor_id: ";
    rosidl_generator_traits::value_to_yaml(msg.motor_id, out);
    out << ", ";
  }

  // member: current_position
  {
    out << "current_position: ";
    rosidl_generator_traits::value_to_yaml(msg.current_position, out);
    out << ", ";
  }

  // member: target_position
  {
    out << "target_position: ";
    rosidl_generator_traits::value_to_yaml(msg.target_position, out);
    out << ", ";
  }

  // member: velocity
  {
    out << "velocity: ";
    rosidl_generator_traits::value_to_yaml(msg.velocity, out);
    out << ", ";
  }

  // member: motion_mode
  {
    out << "motion_mode: ";
    rosidl_generator_traits::value_to_yaml(msg.motion_mode, out);
    out << ", ";
  }

  // member: moving
  {
    out << "moving: ";
    rosidl_generator_traits::value_to_yaml(msg.moving, out);
    out << ", ";
  }

  // member: last_command
  {
    out << "last_command: ";
    rosidl_generator_traits::value_to_yaml(msg.last_command, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const MotorSimulationStatus & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: motor_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "motor_id: ";
    rosidl_generator_traits::value_to_yaml(msg.motor_id, out);
    out << "\n";
  }

  // member: current_position
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "current_position: ";
    rosidl_generator_traits::value_to_yaml(msg.current_position, out);
    out << "\n";
  }

  // member: target_position
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "target_position: ";
    rosidl_generator_traits::value_to_yaml(msg.target_position, out);
    out << "\n";
  }

  // member: velocity
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "velocity: ";
    rosidl_generator_traits::value_to_yaml(msg.velocity, out);
    out << "\n";
  }

  // member: motion_mode
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "motion_mode: ";
    rosidl_generator_traits::value_to_yaml(msg.motion_mode, out);
    out << "\n";
  }

  // member: moving
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "moving: ";
    rosidl_generator_traits::value_to_yaml(msg.moving, out);
    out << "\n";
  }

  // member: last_command
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "last_command: ";
    rosidl_generator_traits::value_to_yaml(msg.last_command, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const MotorSimulationStatus & msg, bool use_flow_style = false)
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

}  // namespace modbus_driver_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use modbus_driver_interfaces::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const modbus_driver_interfaces::msg::MotorSimulationStatus & msg,
  std::ostream & out, size_t indentation = 0)
{
  modbus_driver_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use modbus_driver_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const modbus_driver_interfaces::msg::MotorSimulationStatus & msg)
{
  return modbus_driver_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<modbus_driver_interfaces::msg::MotorSimulationStatus>()
{
  return "modbus_driver_interfaces::msg::MotorSimulationStatus";
}

template<>
inline const char * name<modbus_driver_interfaces::msg::MotorSimulationStatus>()
{
  return "modbus_driver_interfaces/msg/MotorSimulationStatus";
}

template<>
struct has_fixed_size<modbus_driver_interfaces::msg::MotorSimulationStatus>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<modbus_driver_interfaces::msg::MotorSimulationStatus>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<modbus_driver_interfaces::msg::MotorSimulationStatus>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MODBUS_DRIVER_INTERFACES__MSG__DETAIL__MOTOR_SIMULATION_STATUS__TRAITS_HPP_
