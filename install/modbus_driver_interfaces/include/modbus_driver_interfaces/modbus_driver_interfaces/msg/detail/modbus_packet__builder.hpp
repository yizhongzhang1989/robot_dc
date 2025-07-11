// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from modbus_driver_interfaces:msg/ModbusPacket.idl
// generated code does not contain a copyright notice

#ifndef MODBUS_DRIVER_INTERFACES__MSG__DETAIL__MODBUS_PACKET__BUILDER_HPP_
#define MODBUS_DRIVER_INTERFACES__MSG__DETAIL__MODBUS_PACKET__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "modbus_driver_interfaces/msg/detail/modbus_packet__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace modbus_driver_interfaces
{

namespace msg
{

namespace builder
{

class Init_ModbusPacket_values
{
public:
  explicit Init_ModbusPacket_values(::modbus_driver_interfaces::msg::ModbusPacket & msg)
  : msg_(msg)
  {}
  ::modbus_driver_interfaces::msg::ModbusPacket values(::modbus_driver_interfaces::msg::ModbusPacket::_values_type arg)
  {
    msg_.values = std::move(arg);
    return std::move(msg_);
  }

private:
  ::modbus_driver_interfaces::msg::ModbusPacket msg_;
};

class Init_ModbusPacket_count
{
public:
  explicit Init_ModbusPacket_count(::modbus_driver_interfaces::msg::ModbusPacket & msg)
  : msg_(msg)
  {}
  Init_ModbusPacket_values count(::modbus_driver_interfaces::msg::ModbusPacket::_count_type arg)
  {
    msg_.count = std::move(arg);
    return Init_ModbusPacket_values(msg_);
  }

private:
  ::modbus_driver_interfaces::msg::ModbusPacket msg_;
};

class Init_ModbusPacket_address
{
public:
  explicit Init_ModbusPacket_address(::modbus_driver_interfaces::msg::ModbusPacket & msg)
  : msg_(msg)
  {}
  Init_ModbusPacket_count address(::modbus_driver_interfaces::msg::ModbusPacket::_address_type arg)
  {
    msg_.address = std::move(arg);
    return Init_ModbusPacket_count(msg_);
  }

private:
  ::modbus_driver_interfaces::msg::ModbusPacket msg_;
};

class Init_ModbusPacket_slave_id
{
public:
  explicit Init_ModbusPacket_slave_id(::modbus_driver_interfaces::msg::ModbusPacket & msg)
  : msg_(msg)
  {}
  Init_ModbusPacket_address slave_id(::modbus_driver_interfaces::msg::ModbusPacket::_slave_id_type arg)
  {
    msg_.slave_id = std::move(arg);
    return Init_ModbusPacket_address(msg_);
  }

private:
  ::modbus_driver_interfaces::msg::ModbusPacket msg_;
};

class Init_ModbusPacket_function_code
{
public:
  Init_ModbusPacket_function_code()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ModbusPacket_slave_id function_code(::modbus_driver_interfaces::msg::ModbusPacket::_function_code_type arg)
  {
    msg_.function_code = std::move(arg);
    return Init_ModbusPacket_slave_id(msg_);
  }

private:
  ::modbus_driver_interfaces::msg::ModbusPacket msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::modbus_driver_interfaces::msg::ModbusPacket>()
{
  return modbus_driver_interfaces::msg::builder::Init_ModbusPacket_function_code();
}

}  // namespace modbus_driver_interfaces

#endif  // MODBUS_DRIVER_INTERFACES__MSG__DETAIL__MODBUS_PACKET__BUILDER_HPP_
