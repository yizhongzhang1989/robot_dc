// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from modbus_driver_interfaces:msg/ModbusPacket.idl
// generated code does not contain a copyright notice

#ifndef MODBUS_DRIVER_INTERFACES__MSG__DETAIL__MODBUS_PACKET__TRAITS_HPP_
#define MODBUS_DRIVER_INTERFACES__MSG__DETAIL__MODBUS_PACKET__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "modbus_driver_interfaces/msg/detail/modbus_packet__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace modbus_driver_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const ModbusPacket & msg,
  std::ostream & out)
{
  out << "{";
  // member: function_code
  {
    out << "function_code: ";
    rosidl_generator_traits::value_to_yaml(msg.function_code, out);
    out << ", ";
  }

  // member: slave_id
  {
    out << "slave_id: ";
    rosidl_generator_traits::value_to_yaml(msg.slave_id, out);
    out << ", ";
  }

  // member: address
  {
    out << "address: ";
    rosidl_generator_traits::value_to_yaml(msg.address, out);
    out << ", ";
  }

  // member: count
  {
    out << "count: ";
    rosidl_generator_traits::value_to_yaml(msg.count, out);
    out << ", ";
  }

  // member: values
  {
    if (msg.values.size() == 0) {
      out << "values: []";
    } else {
      out << "values: [";
      size_t pending_items = msg.values.size();
      for (auto item : msg.values) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ModbusPacket & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: function_code
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "function_code: ";
    rosidl_generator_traits::value_to_yaml(msg.function_code, out);
    out << "\n";
  }

  // member: slave_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "slave_id: ";
    rosidl_generator_traits::value_to_yaml(msg.slave_id, out);
    out << "\n";
  }

  // member: address
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "address: ";
    rosidl_generator_traits::value_to_yaml(msg.address, out);
    out << "\n";
  }

  // member: count
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "count: ";
    rosidl_generator_traits::value_to_yaml(msg.count, out);
    out << "\n";
  }

  // member: values
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.values.size() == 0) {
      out << "values: []\n";
    } else {
      out << "values:\n";
      for (auto item : msg.values) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ModbusPacket & msg, bool use_flow_style = false)
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
  const modbus_driver_interfaces::msg::ModbusPacket & msg,
  std::ostream & out, size_t indentation = 0)
{
  modbus_driver_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use modbus_driver_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const modbus_driver_interfaces::msg::ModbusPacket & msg)
{
  return modbus_driver_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<modbus_driver_interfaces::msg::ModbusPacket>()
{
  return "modbus_driver_interfaces::msg::ModbusPacket";
}

template<>
inline const char * name<modbus_driver_interfaces::msg::ModbusPacket>()
{
  return "modbus_driver_interfaces/msg/ModbusPacket";
}

template<>
struct has_fixed_size<modbus_driver_interfaces::msg::ModbusPacket>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<modbus_driver_interfaces::msg::ModbusPacket>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<modbus_driver_interfaces::msg::ModbusPacket>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MODBUS_DRIVER_INTERFACES__MSG__DETAIL__MODBUS_PACKET__TRAITS_HPP_
