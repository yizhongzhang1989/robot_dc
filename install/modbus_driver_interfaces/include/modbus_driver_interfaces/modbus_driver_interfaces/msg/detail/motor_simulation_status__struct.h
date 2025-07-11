// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from modbus_driver_interfaces:msg/MotorSimulationStatus.idl
// generated code does not contain a copyright notice

#ifndef MODBUS_DRIVER_INTERFACES__MSG__DETAIL__MOTOR_SIMULATION_STATUS__STRUCT_H_
#define MODBUS_DRIVER_INTERFACES__MSG__DETAIL__MOTOR_SIMULATION_STATUS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'last_command'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/MotorSimulationStatus in the package modbus_driver_interfaces.
typedef struct modbus_driver_interfaces__msg__MotorSimulationStatus
{
  int32_t motor_id;
  float current_position;
  float target_position;
  float velocity;
  int32_t motion_mode;
  bool moving;
  /// e.g., "move_abs", "jog_left", etc.
  rosidl_runtime_c__String last_command;
} modbus_driver_interfaces__msg__MotorSimulationStatus;

// Struct for a sequence of modbus_driver_interfaces__msg__MotorSimulationStatus.
typedef struct modbus_driver_interfaces__msg__MotorSimulationStatus__Sequence
{
  modbus_driver_interfaces__msg__MotorSimulationStatus * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} modbus_driver_interfaces__msg__MotorSimulationStatus__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MODBUS_DRIVER_INTERFACES__MSG__DETAIL__MOTOR_SIMULATION_STATUS__STRUCT_H_
