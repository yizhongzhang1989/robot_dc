// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from modbus_driver_interfaces:msg/MotorSimulationStatus.idl
// generated code does not contain a copyright notice
#include "modbus_driver_interfaces/msg/detail/motor_simulation_status__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `last_command`
#include "rosidl_runtime_c/string_functions.h"

bool
modbus_driver_interfaces__msg__MotorSimulationStatus__init(modbus_driver_interfaces__msg__MotorSimulationStatus * msg)
{
  if (!msg) {
    return false;
  }
  // motor_id
  // current_position
  // target_position
  // velocity
  // motion_mode
  // moving
  // last_command
  if (!rosidl_runtime_c__String__init(&msg->last_command)) {
    modbus_driver_interfaces__msg__MotorSimulationStatus__fini(msg);
    return false;
  }
  return true;
}

void
modbus_driver_interfaces__msg__MotorSimulationStatus__fini(modbus_driver_interfaces__msg__MotorSimulationStatus * msg)
{
  if (!msg) {
    return;
  }
  // motor_id
  // current_position
  // target_position
  // velocity
  // motion_mode
  // moving
  // last_command
  rosidl_runtime_c__String__fini(&msg->last_command);
}

bool
modbus_driver_interfaces__msg__MotorSimulationStatus__are_equal(const modbus_driver_interfaces__msg__MotorSimulationStatus * lhs, const modbus_driver_interfaces__msg__MotorSimulationStatus * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // motor_id
  if (lhs->motor_id != rhs->motor_id) {
    return false;
  }
  // current_position
  if (lhs->current_position != rhs->current_position) {
    return false;
  }
  // target_position
  if (lhs->target_position != rhs->target_position) {
    return false;
  }
  // velocity
  if (lhs->velocity != rhs->velocity) {
    return false;
  }
  // motion_mode
  if (lhs->motion_mode != rhs->motion_mode) {
    return false;
  }
  // moving
  if (lhs->moving != rhs->moving) {
    return false;
  }
  // last_command
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->last_command), &(rhs->last_command)))
  {
    return false;
  }
  return true;
}

bool
modbus_driver_interfaces__msg__MotorSimulationStatus__copy(
  const modbus_driver_interfaces__msg__MotorSimulationStatus * input,
  modbus_driver_interfaces__msg__MotorSimulationStatus * output)
{
  if (!input || !output) {
    return false;
  }
  // motor_id
  output->motor_id = input->motor_id;
  // current_position
  output->current_position = input->current_position;
  // target_position
  output->target_position = input->target_position;
  // velocity
  output->velocity = input->velocity;
  // motion_mode
  output->motion_mode = input->motion_mode;
  // moving
  output->moving = input->moving;
  // last_command
  if (!rosidl_runtime_c__String__copy(
      &(input->last_command), &(output->last_command)))
  {
    return false;
  }
  return true;
}

modbus_driver_interfaces__msg__MotorSimulationStatus *
modbus_driver_interfaces__msg__MotorSimulationStatus__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  modbus_driver_interfaces__msg__MotorSimulationStatus * msg = (modbus_driver_interfaces__msg__MotorSimulationStatus *)allocator.allocate(sizeof(modbus_driver_interfaces__msg__MotorSimulationStatus), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(modbus_driver_interfaces__msg__MotorSimulationStatus));
  bool success = modbus_driver_interfaces__msg__MotorSimulationStatus__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
modbus_driver_interfaces__msg__MotorSimulationStatus__destroy(modbus_driver_interfaces__msg__MotorSimulationStatus * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    modbus_driver_interfaces__msg__MotorSimulationStatus__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
modbus_driver_interfaces__msg__MotorSimulationStatus__Sequence__init(modbus_driver_interfaces__msg__MotorSimulationStatus__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  modbus_driver_interfaces__msg__MotorSimulationStatus * data = NULL;

  if (size) {
    data = (modbus_driver_interfaces__msg__MotorSimulationStatus *)allocator.zero_allocate(size, sizeof(modbus_driver_interfaces__msg__MotorSimulationStatus), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = modbus_driver_interfaces__msg__MotorSimulationStatus__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        modbus_driver_interfaces__msg__MotorSimulationStatus__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
modbus_driver_interfaces__msg__MotorSimulationStatus__Sequence__fini(modbus_driver_interfaces__msg__MotorSimulationStatus__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      modbus_driver_interfaces__msg__MotorSimulationStatus__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

modbus_driver_interfaces__msg__MotorSimulationStatus__Sequence *
modbus_driver_interfaces__msg__MotorSimulationStatus__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  modbus_driver_interfaces__msg__MotorSimulationStatus__Sequence * array = (modbus_driver_interfaces__msg__MotorSimulationStatus__Sequence *)allocator.allocate(sizeof(modbus_driver_interfaces__msg__MotorSimulationStatus__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = modbus_driver_interfaces__msg__MotorSimulationStatus__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
modbus_driver_interfaces__msg__MotorSimulationStatus__Sequence__destroy(modbus_driver_interfaces__msg__MotorSimulationStatus__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    modbus_driver_interfaces__msg__MotorSimulationStatus__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
modbus_driver_interfaces__msg__MotorSimulationStatus__Sequence__are_equal(const modbus_driver_interfaces__msg__MotorSimulationStatus__Sequence * lhs, const modbus_driver_interfaces__msg__MotorSimulationStatus__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!modbus_driver_interfaces__msg__MotorSimulationStatus__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
modbus_driver_interfaces__msg__MotorSimulationStatus__Sequence__copy(
  const modbus_driver_interfaces__msg__MotorSimulationStatus__Sequence * input,
  modbus_driver_interfaces__msg__MotorSimulationStatus__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(modbus_driver_interfaces__msg__MotorSimulationStatus);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    modbus_driver_interfaces__msg__MotorSimulationStatus * data =
      (modbus_driver_interfaces__msg__MotorSimulationStatus *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!modbus_driver_interfaces__msg__MotorSimulationStatus__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          modbus_driver_interfaces__msg__MotorSimulationStatus__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!modbus_driver_interfaces__msg__MotorSimulationStatus__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
