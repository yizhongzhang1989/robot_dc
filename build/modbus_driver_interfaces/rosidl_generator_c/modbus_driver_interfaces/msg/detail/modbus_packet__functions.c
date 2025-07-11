// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from modbus_driver_interfaces:msg/ModbusPacket.idl
// generated code does not contain a copyright notice
#include "modbus_driver_interfaces/msg/detail/modbus_packet__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `values`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
modbus_driver_interfaces__msg__ModbusPacket__init(modbus_driver_interfaces__msg__ModbusPacket * msg)
{
  if (!msg) {
    return false;
  }
  // function_code
  // slave_id
  // address
  // count
  // values
  if (!rosidl_runtime_c__uint16__Sequence__init(&msg->values, 0)) {
    modbus_driver_interfaces__msg__ModbusPacket__fini(msg);
    return false;
  }
  return true;
}

void
modbus_driver_interfaces__msg__ModbusPacket__fini(modbus_driver_interfaces__msg__ModbusPacket * msg)
{
  if (!msg) {
    return;
  }
  // function_code
  // slave_id
  // address
  // count
  // values
  rosidl_runtime_c__uint16__Sequence__fini(&msg->values);
}

bool
modbus_driver_interfaces__msg__ModbusPacket__are_equal(const modbus_driver_interfaces__msg__ModbusPacket * lhs, const modbus_driver_interfaces__msg__ModbusPacket * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // function_code
  if (lhs->function_code != rhs->function_code) {
    return false;
  }
  // slave_id
  if (lhs->slave_id != rhs->slave_id) {
    return false;
  }
  // address
  if (lhs->address != rhs->address) {
    return false;
  }
  // count
  if (lhs->count != rhs->count) {
    return false;
  }
  // values
  if (!rosidl_runtime_c__uint16__Sequence__are_equal(
      &(lhs->values), &(rhs->values)))
  {
    return false;
  }
  return true;
}

bool
modbus_driver_interfaces__msg__ModbusPacket__copy(
  const modbus_driver_interfaces__msg__ModbusPacket * input,
  modbus_driver_interfaces__msg__ModbusPacket * output)
{
  if (!input || !output) {
    return false;
  }
  // function_code
  output->function_code = input->function_code;
  // slave_id
  output->slave_id = input->slave_id;
  // address
  output->address = input->address;
  // count
  output->count = input->count;
  // values
  if (!rosidl_runtime_c__uint16__Sequence__copy(
      &(input->values), &(output->values)))
  {
    return false;
  }
  return true;
}

modbus_driver_interfaces__msg__ModbusPacket *
modbus_driver_interfaces__msg__ModbusPacket__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  modbus_driver_interfaces__msg__ModbusPacket * msg = (modbus_driver_interfaces__msg__ModbusPacket *)allocator.allocate(sizeof(modbus_driver_interfaces__msg__ModbusPacket), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(modbus_driver_interfaces__msg__ModbusPacket));
  bool success = modbus_driver_interfaces__msg__ModbusPacket__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
modbus_driver_interfaces__msg__ModbusPacket__destroy(modbus_driver_interfaces__msg__ModbusPacket * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    modbus_driver_interfaces__msg__ModbusPacket__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
modbus_driver_interfaces__msg__ModbusPacket__Sequence__init(modbus_driver_interfaces__msg__ModbusPacket__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  modbus_driver_interfaces__msg__ModbusPacket * data = NULL;

  if (size) {
    data = (modbus_driver_interfaces__msg__ModbusPacket *)allocator.zero_allocate(size, sizeof(modbus_driver_interfaces__msg__ModbusPacket), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = modbus_driver_interfaces__msg__ModbusPacket__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        modbus_driver_interfaces__msg__ModbusPacket__fini(&data[i - 1]);
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
modbus_driver_interfaces__msg__ModbusPacket__Sequence__fini(modbus_driver_interfaces__msg__ModbusPacket__Sequence * array)
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
      modbus_driver_interfaces__msg__ModbusPacket__fini(&array->data[i]);
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

modbus_driver_interfaces__msg__ModbusPacket__Sequence *
modbus_driver_interfaces__msg__ModbusPacket__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  modbus_driver_interfaces__msg__ModbusPacket__Sequence * array = (modbus_driver_interfaces__msg__ModbusPacket__Sequence *)allocator.allocate(sizeof(modbus_driver_interfaces__msg__ModbusPacket__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = modbus_driver_interfaces__msg__ModbusPacket__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
modbus_driver_interfaces__msg__ModbusPacket__Sequence__destroy(modbus_driver_interfaces__msg__ModbusPacket__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    modbus_driver_interfaces__msg__ModbusPacket__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
modbus_driver_interfaces__msg__ModbusPacket__Sequence__are_equal(const modbus_driver_interfaces__msg__ModbusPacket__Sequence * lhs, const modbus_driver_interfaces__msg__ModbusPacket__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!modbus_driver_interfaces__msg__ModbusPacket__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
modbus_driver_interfaces__msg__ModbusPacket__Sequence__copy(
  const modbus_driver_interfaces__msg__ModbusPacket__Sequence * input,
  modbus_driver_interfaces__msg__ModbusPacket__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(modbus_driver_interfaces__msg__ModbusPacket);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    modbus_driver_interfaces__msg__ModbusPacket * data =
      (modbus_driver_interfaces__msg__ModbusPacket *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!modbus_driver_interfaces__msg__ModbusPacket__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          modbus_driver_interfaces__msg__ModbusPacket__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!modbus_driver_interfaces__msg__ModbusPacket__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
