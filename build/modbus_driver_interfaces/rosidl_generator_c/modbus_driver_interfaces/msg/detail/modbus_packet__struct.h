// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from modbus_driver_interfaces:msg/ModbusPacket.idl
// generated code does not contain a copyright notice

#ifndef MODBUS_DRIVER_INTERFACES__MSG__DETAIL__MODBUS_PACKET__STRUCT_H_
#define MODBUS_DRIVER_INTERFACES__MSG__DETAIL__MODBUS_PACKET__STRUCT_H_

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

/// Struct defined in msg/ModbusPacket in the package modbus_driver_interfaces.
typedef struct modbus_driver_interfaces__msg__ModbusPacket
{
  uint8_t function_code;
  uint8_t slave_id;
  uint16_t address;
  uint16_t count;
  rosidl_runtime_c__uint16__Sequence values;
} modbus_driver_interfaces__msg__ModbusPacket;

// Struct for a sequence of modbus_driver_interfaces__msg__ModbusPacket.
typedef struct modbus_driver_interfaces__msg__ModbusPacket__Sequence
{
  modbus_driver_interfaces__msg__ModbusPacket * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} modbus_driver_interfaces__msg__ModbusPacket__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MODBUS_DRIVER_INTERFACES__MSG__DETAIL__MODBUS_PACKET__STRUCT_H_
