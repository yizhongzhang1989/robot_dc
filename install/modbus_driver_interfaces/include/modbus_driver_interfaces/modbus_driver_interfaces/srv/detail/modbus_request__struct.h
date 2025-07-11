// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from modbus_driver_interfaces:srv/ModbusRequest.idl
// generated code does not contain a copyright notice

#ifndef MODBUS_DRIVER_INTERFACES__SRV__DETAIL__MODBUS_REQUEST__STRUCT_H_
#define MODBUS_DRIVER_INTERFACES__SRV__DETAIL__MODBUS_REQUEST__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'values'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in srv/ModbusRequest in the package modbus_driver_interfaces.
typedef struct modbus_driver_interfaces__srv__ModbusRequest_Request
{
  uint8_t function_code;
  uint8_t slave_id;
  uint16_t address;
  uint16_t count;
  rosidl_runtime_c__uint16__Sequence values;
  uint16_t seq_id;
} modbus_driver_interfaces__srv__ModbusRequest_Request;

// Struct for a sequence of modbus_driver_interfaces__srv__ModbusRequest_Request.
typedef struct modbus_driver_interfaces__srv__ModbusRequest_Request__Sequence
{
  modbus_driver_interfaces__srv__ModbusRequest_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} modbus_driver_interfaces__srv__ModbusRequest_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'response'
// already included above
// #include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in srv/ModbusRequest in the package modbus_driver_interfaces.
typedef struct modbus_driver_interfaces__srv__ModbusRequest_Response
{
  bool success;
  rosidl_runtime_c__uint16__Sequence response;
  uint16_t ack;
} modbus_driver_interfaces__srv__ModbusRequest_Response;

// Struct for a sequence of modbus_driver_interfaces__srv__ModbusRequest_Response.
typedef struct modbus_driver_interfaces__srv__ModbusRequest_Response__Sequence
{
  modbus_driver_interfaces__srv__ModbusRequest_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} modbus_driver_interfaces__srv__ModbusRequest_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MODBUS_DRIVER_INTERFACES__SRV__DETAIL__MODBUS_REQUEST__STRUCT_H_
