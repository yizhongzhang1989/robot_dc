// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from modbus_driver_interfaces:msg/MotorSimulationStatus.idl
// generated code does not contain a copyright notice

#ifndef MODBUS_DRIVER_INTERFACES__MSG__DETAIL__MOTOR_SIMULATION_STATUS__FUNCTIONS_H_
#define MODBUS_DRIVER_INTERFACES__MSG__DETAIL__MOTOR_SIMULATION_STATUS__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "modbus_driver_interfaces/msg/rosidl_generator_c__visibility_control.h"

#include "modbus_driver_interfaces/msg/detail/motor_simulation_status__struct.h"

/// Initialize msg/MotorSimulationStatus message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * modbus_driver_interfaces__msg__MotorSimulationStatus
 * )) before or use
 * modbus_driver_interfaces__msg__MotorSimulationStatus__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_modbus_driver_interfaces
bool
modbus_driver_interfaces__msg__MotorSimulationStatus__init(modbus_driver_interfaces__msg__MotorSimulationStatus * msg);

/// Finalize msg/MotorSimulationStatus message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_modbus_driver_interfaces
void
modbus_driver_interfaces__msg__MotorSimulationStatus__fini(modbus_driver_interfaces__msg__MotorSimulationStatus * msg);

/// Create msg/MotorSimulationStatus message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * modbus_driver_interfaces__msg__MotorSimulationStatus__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_modbus_driver_interfaces
modbus_driver_interfaces__msg__MotorSimulationStatus *
modbus_driver_interfaces__msg__MotorSimulationStatus__create();

/// Destroy msg/MotorSimulationStatus message.
/**
 * It calls
 * modbus_driver_interfaces__msg__MotorSimulationStatus__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_modbus_driver_interfaces
void
modbus_driver_interfaces__msg__MotorSimulationStatus__destroy(modbus_driver_interfaces__msg__MotorSimulationStatus * msg);

/// Check for msg/MotorSimulationStatus message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_modbus_driver_interfaces
bool
modbus_driver_interfaces__msg__MotorSimulationStatus__are_equal(const modbus_driver_interfaces__msg__MotorSimulationStatus * lhs, const modbus_driver_interfaces__msg__MotorSimulationStatus * rhs);

/// Copy a msg/MotorSimulationStatus message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_modbus_driver_interfaces
bool
modbus_driver_interfaces__msg__MotorSimulationStatus__copy(
  const modbus_driver_interfaces__msg__MotorSimulationStatus * input,
  modbus_driver_interfaces__msg__MotorSimulationStatus * output);

/// Initialize array of msg/MotorSimulationStatus messages.
/**
 * It allocates the memory for the number of elements and calls
 * modbus_driver_interfaces__msg__MotorSimulationStatus__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_modbus_driver_interfaces
bool
modbus_driver_interfaces__msg__MotorSimulationStatus__Sequence__init(modbus_driver_interfaces__msg__MotorSimulationStatus__Sequence * array, size_t size);

/// Finalize array of msg/MotorSimulationStatus messages.
/**
 * It calls
 * modbus_driver_interfaces__msg__MotorSimulationStatus__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_modbus_driver_interfaces
void
modbus_driver_interfaces__msg__MotorSimulationStatus__Sequence__fini(modbus_driver_interfaces__msg__MotorSimulationStatus__Sequence * array);

/// Create array of msg/MotorSimulationStatus messages.
/**
 * It allocates the memory for the array and calls
 * modbus_driver_interfaces__msg__MotorSimulationStatus__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_modbus_driver_interfaces
modbus_driver_interfaces__msg__MotorSimulationStatus__Sequence *
modbus_driver_interfaces__msg__MotorSimulationStatus__Sequence__create(size_t size);

/// Destroy array of msg/MotorSimulationStatus messages.
/**
 * It calls
 * modbus_driver_interfaces__msg__MotorSimulationStatus__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_modbus_driver_interfaces
void
modbus_driver_interfaces__msg__MotorSimulationStatus__Sequence__destroy(modbus_driver_interfaces__msg__MotorSimulationStatus__Sequence * array);

/// Check for msg/MotorSimulationStatus message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_modbus_driver_interfaces
bool
modbus_driver_interfaces__msg__MotorSimulationStatus__Sequence__are_equal(const modbus_driver_interfaces__msg__MotorSimulationStatus__Sequence * lhs, const modbus_driver_interfaces__msg__MotorSimulationStatus__Sequence * rhs);

/// Copy an array of msg/MotorSimulationStatus messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_modbus_driver_interfaces
bool
modbus_driver_interfaces__msg__MotorSimulationStatus__Sequence__copy(
  const modbus_driver_interfaces__msg__MotorSimulationStatus__Sequence * input,
  modbus_driver_interfaces__msg__MotorSimulationStatus__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // MODBUS_DRIVER_INTERFACES__MSG__DETAIL__MOTOR_SIMULATION_STATUS__FUNCTIONS_H_
