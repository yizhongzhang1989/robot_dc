// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from modbus_driver_interfaces:srv/ModbusRequest.idl
// generated code does not contain a copyright notice

#ifndef MODBUS_DRIVER_INTERFACES__SRV__DETAIL__MODBUS_REQUEST__BUILDER_HPP_
#define MODBUS_DRIVER_INTERFACES__SRV__DETAIL__MODBUS_REQUEST__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "modbus_driver_interfaces/srv/detail/modbus_request__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace modbus_driver_interfaces
{

namespace srv
{

namespace builder
{

class Init_ModbusRequest_Request_seq_id
{
public:
  explicit Init_ModbusRequest_Request_seq_id(::modbus_driver_interfaces::srv::ModbusRequest_Request & msg)
  : msg_(msg)
  {}
  ::modbus_driver_interfaces::srv::ModbusRequest_Request seq_id(::modbus_driver_interfaces::srv::ModbusRequest_Request::_seq_id_type arg)
  {
    msg_.seq_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::modbus_driver_interfaces::srv::ModbusRequest_Request msg_;
};

class Init_ModbusRequest_Request_values
{
public:
  explicit Init_ModbusRequest_Request_values(::modbus_driver_interfaces::srv::ModbusRequest_Request & msg)
  : msg_(msg)
  {}
  Init_ModbusRequest_Request_seq_id values(::modbus_driver_interfaces::srv::ModbusRequest_Request::_values_type arg)
  {
    msg_.values = std::move(arg);
    return Init_ModbusRequest_Request_seq_id(msg_);
  }

private:
  ::modbus_driver_interfaces::srv::ModbusRequest_Request msg_;
};

class Init_ModbusRequest_Request_count
{
public:
  explicit Init_ModbusRequest_Request_count(::modbus_driver_interfaces::srv::ModbusRequest_Request & msg)
  : msg_(msg)
  {}
  Init_ModbusRequest_Request_values count(::modbus_driver_interfaces::srv::ModbusRequest_Request::_count_type arg)
  {
    msg_.count = std::move(arg);
    return Init_ModbusRequest_Request_values(msg_);
  }

private:
  ::modbus_driver_interfaces::srv::ModbusRequest_Request msg_;
};

class Init_ModbusRequest_Request_address
{
public:
  explicit Init_ModbusRequest_Request_address(::modbus_driver_interfaces::srv::ModbusRequest_Request & msg)
  : msg_(msg)
  {}
  Init_ModbusRequest_Request_count address(::modbus_driver_interfaces::srv::ModbusRequest_Request::_address_type arg)
  {
    msg_.address = std::move(arg);
    return Init_ModbusRequest_Request_count(msg_);
  }

private:
  ::modbus_driver_interfaces::srv::ModbusRequest_Request msg_;
};

class Init_ModbusRequest_Request_slave_id
{
public:
  explicit Init_ModbusRequest_Request_slave_id(::modbus_driver_interfaces::srv::ModbusRequest_Request & msg)
  : msg_(msg)
  {}
  Init_ModbusRequest_Request_address slave_id(::modbus_driver_interfaces::srv::ModbusRequest_Request::_slave_id_type arg)
  {
    msg_.slave_id = std::move(arg);
    return Init_ModbusRequest_Request_address(msg_);
  }

private:
  ::modbus_driver_interfaces::srv::ModbusRequest_Request msg_;
};

class Init_ModbusRequest_Request_function_code
{
public:
  Init_ModbusRequest_Request_function_code()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ModbusRequest_Request_slave_id function_code(::modbus_driver_interfaces::srv::ModbusRequest_Request::_function_code_type arg)
  {
    msg_.function_code = std::move(arg);
    return Init_ModbusRequest_Request_slave_id(msg_);
  }

private:
  ::modbus_driver_interfaces::srv::ModbusRequest_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::modbus_driver_interfaces::srv::ModbusRequest_Request>()
{
  return modbus_driver_interfaces::srv::builder::Init_ModbusRequest_Request_function_code();
}

}  // namespace modbus_driver_interfaces


namespace modbus_driver_interfaces
{

namespace srv
{

namespace builder
{

class Init_ModbusRequest_Response_ack
{
public:
  explicit Init_ModbusRequest_Response_ack(::modbus_driver_interfaces::srv::ModbusRequest_Response & msg)
  : msg_(msg)
  {}
  ::modbus_driver_interfaces::srv::ModbusRequest_Response ack(::modbus_driver_interfaces::srv::ModbusRequest_Response::_ack_type arg)
  {
    msg_.ack = std::move(arg);
    return std::move(msg_);
  }

private:
  ::modbus_driver_interfaces::srv::ModbusRequest_Response msg_;
};

class Init_ModbusRequest_Response_response
{
public:
  explicit Init_ModbusRequest_Response_response(::modbus_driver_interfaces::srv::ModbusRequest_Response & msg)
  : msg_(msg)
  {}
  Init_ModbusRequest_Response_ack response(::modbus_driver_interfaces::srv::ModbusRequest_Response::_response_type arg)
  {
    msg_.response = std::move(arg);
    return Init_ModbusRequest_Response_ack(msg_);
  }

private:
  ::modbus_driver_interfaces::srv::ModbusRequest_Response msg_;
};

class Init_ModbusRequest_Response_success
{
public:
  Init_ModbusRequest_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ModbusRequest_Response_response success(::modbus_driver_interfaces::srv::ModbusRequest_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_ModbusRequest_Response_response(msg_);
  }

private:
  ::modbus_driver_interfaces::srv::ModbusRequest_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::modbus_driver_interfaces::srv::ModbusRequest_Response>()
{
  return modbus_driver_interfaces::srv::builder::Init_ModbusRequest_Response_success();
}

}  // namespace modbus_driver_interfaces

#endif  // MODBUS_DRIVER_INTERFACES__SRV__DETAIL__MODBUS_REQUEST__BUILDER_HPP_
