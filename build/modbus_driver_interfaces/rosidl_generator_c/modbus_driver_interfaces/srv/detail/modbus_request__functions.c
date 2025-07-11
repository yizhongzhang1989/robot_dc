// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from modbus_driver_interfaces:srv/ModbusRequest.idl
// generated code does not contain a copyright notice
#include "modbus_driver_interfaces/srv/detail/modbus_request__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

// Include directives for member types
// Member `values`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
modbus_driver_interfaces__srv__ModbusRequest_Request__init(modbus_driver_interfaces__srv__ModbusRequest_Request * msg)
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
    modbus_driver_interfaces__srv__ModbusRequest_Request__fini(msg);
    return false;
  }
  // seq_id
  return true;
}

void
modbus_driver_interfaces__srv__ModbusRequest_Request__fini(modbus_driver_interfaces__srv__ModbusRequest_Request * msg)
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
  // seq_id
}

bool
modbus_driver_interfaces__srv__ModbusRequest_Request__are_equal(const modbus_driver_interfaces__srv__ModbusRequest_Request * lhs, const modbus_driver_interfaces__srv__ModbusRequest_Request * rhs)
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
  // seq_id
  if (lhs->seq_id != rhs->seq_id) {
    return false;
  }
  return true;
}

bool
modbus_driver_interfaces__srv__ModbusRequest_Request__copy(
  const modbus_driver_interfaces__srv__ModbusRequest_Request * input,
  modbus_driver_interfaces__srv__ModbusRequest_Request * output)
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
  // seq_id
  output->seq_id = input->seq_id;
  return true;
}

modbus_driver_interfaces__srv__ModbusRequest_Request *
modbus_driver_interfaces__srv__ModbusRequest_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  modbus_driver_interfaces__srv__ModbusRequest_Request * msg = (modbus_driver_interfaces__srv__ModbusRequest_Request *)allocator.allocate(sizeof(modbus_driver_interfaces__srv__ModbusRequest_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(modbus_driver_interfaces__srv__ModbusRequest_Request));
  bool success = modbus_driver_interfaces__srv__ModbusRequest_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
modbus_driver_interfaces__srv__ModbusRequest_Request__destroy(modbus_driver_interfaces__srv__ModbusRequest_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    modbus_driver_interfaces__srv__ModbusRequest_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
modbus_driver_interfaces__srv__ModbusRequest_Request__Sequence__init(modbus_driver_interfaces__srv__ModbusRequest_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  modbus_driver_interfaces__srv__ModbusRequest_Request * data = NULL;

  if (size) {
    data = (modbus_driver_interfaces__srv__ModbusRequest_Request *)allocator.zero_allocate(size, sizeof(modbus_driver_interfaces__srv__ModbusRequest_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = modbus_driver_interfaces__srv__ModbusRequest_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        modbus_driver_interfaces__srv__ModbusRequest_Request__fini(&data[i - 1]);
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
modbus_driver_interfaces__srv__ModbusRequest_Request__Sequence__fini(modbus_driver_interfaces__srv__ModbusRequest_Request__Sequence * array)
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
      modbus_driver_interfaces__srv__ModbusRequest_Request__fini(&array->data[i]);
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

modbus_driver_interfaces__srv__ModbusRequest_Request__Sequence *
modbus_driver_interfaces__srv__ModbusRequest_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  modbus_driver_interfaces__srv__ModbusRequest_Request__Sequence * array = (modbus_driver_interfaces__srv__ModbusRequest_Request__Sequence *)allocator.allocate(sizeof(modbus_driver_interfaces__srv__ModbusRequest_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = modbus_driver_interfaces__srv__ModbusRequest_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
modbus_driver_interfaces__srv__ModbusRequest_Request__Sequence__destroy(modbus_driver_interfaces__srv__ModbusRequest_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    modbus_driver_interfaces__srv__ModbusRequest_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
modbus_driver_interfaces__srv__ModbusRequest_Request__Sequence__are_equal(const modbus_driver_interfaces__srv__ModbusRequest_Request__Sequence * lhs, const modbus_driver_interfaces__srv__ModbusRequest_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!modbus_driver_interfaces__srv__ModbusRequest_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
modbus_driver_interfaces__srv__ModbusRequest_Request__Sequence__copy(
  const modbus_driver_interfaces__srv__ModbusRequest_Request__Sequence * input,
  modbus_driver_interfaces__srv__ModbusRequest_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(modbus_driver_interfaces__srv__ModbusRequest_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    modbus_driver_interfaces__srv__ModbusRequest_Request * data =
      (modbus_driver_interfaces__srv__ModbusRequest_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!modbus_driver_interfaces__srv__ModbusRequest_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          modbus_driver_interfaces__srv__ModbusRequest_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!modbus_driver_interfaces__srv__ModbusRequest_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `response`
// already included above
// #include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
modbus_driver_interfaces__srv__ModbusRequest_Response__init(modbus_driver_interfaces__srv__ModbusRequest_Response * msg)
{
  if (!msg) {
    return false;
  }
  // success
  // response
  if (!rosidl_runtime_c__uint16__Sequence__init(&msg->response, 0)) {
    modbus_driver_interfaces__srv__ModbusRequest_Response__fini(msg);
    return false;
  }
  // ack
  return true;
}

void
modbus_driver_interfaces__srv__ModbusRequest_Response__fini(modbus_driver_interfaces__srv__ModbusRequest_Response * msg)
{
  if (!msg) {
    return;
  }
  // success
  // response
  rosidl_runtime_c__uint16__Sequence__fini(&msg->response);
  // ack
}

bool
modbus_driver_interfaces__srv__ModbusRequest_Response__are_equal(const modbus_driver_interfaces__srv__ModbusRequest_Response * lhs, const modbus_driver_interfaces__srv__ModbusRequest_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // success
  if (lhs->success != rhs->success) {
    return false;
  }
  // response
  if (!rosidl_runtime_c__uint16__Sequence__are_equal(
      &(lhs->response), &(rhs->response)))
  {
    return false;
  }
  // ack
  if (lhs->ack != rhs->ack) {
    return false;
  }
  return true;
}

bool
modbus_driver_interfaces__srv__ModbusRequest_Response__copy(
  const modbus_driver_interfaces__srv__ModbusRequest_Response * input,
  modbus_driver_interfaces__srv__ModbusRequest_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // success
  output->success = input->success;
  // response
  if (!rosidl_runtime_c__uint16__Sequence__copy(
      &(input->response), &(output->response)))
  {
    return false;
  }
  // ack
  output->ack = input->ack;
  return true;
}

modbus_driver_interfaces__srv__ModbusRequest_Response *
modbus_driver_interfaces__srv__ModbusRequest_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  modbus_driver_interfaces__srv__ModbusRequest_Response * msg = (modbus_driver_interfaces__srv__ModbusRequest_Response *)allocator.allocate(sizeof(modbus_driver_interfaces__srv__ModbusRequest_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(modbus_driver_interfaces__srv__ModbusRequest_Response));
  bool success = modbus_driver_interfaces__srv__ModbusRequest_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
modbus_driver_interfaces__srv__ModbusRequest_Response__destroy(modbus_driver_interfaces__srv__ModbusRequest_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    modbus_driver_interfaces__srv__ModbusRequest_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
modbus_driver_interfaces__srv__ModbusRequest_Response__Sequence__init(modbus_driver_interfaces__srv__ModbusRequest_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  modbus_driver_interfaces__srv__ModbusRequest_Response * data = NULL;

  if (size) {
    data = (modbus_driver_interfaces__srv__ModbusRequest_Response *)allocator.zero_allocate(size, sizeof(modbus_driver_interfaces__srv__ModbusRequest_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = modbus_driver_interfaces__srv__ModbusRequest_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        modbus_driver_interfaces__srv__ModbusRequest_Response__fini(&data[i - 1]);
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
modbus_driver_interfaces__srv__ModbusRequest_Response__Sequence__fini(modbus_driver_interfaces__srv__ModbusRequest_Response__Sequence * array)
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
      modbus_driver_interfaces__srv__ModbusRequest_Response__fini(&array->data[i]);
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

modbus_driver_interfaces__srv__ModbusRequest_Response__Sequence *
modbus_driver_interfaces__srv__ModbusRequest_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  modbus_driver_interfaces__srv__ModbusRequest_Response__Sequence * array = (modbus_driver_interfaces__srv__ModbusRequest_Response__Sequence *)allocator.allocate(sizeof(modbus_driver_interfaces__srv__ModbusRequest_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = modbus_driver_interfaces__srv__ModbusRequest_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
modbus_driver_interfaces__srv__ModbusRequest_Response__Sequence__destroy(modbus_driver_interfaces__srv__ModbusRequest_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    modbus_driver_interfaces__srv__ModbusRequest_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
modbus_driver_interfaces__srv__ModbusRequest_Response__Sequence__are_equal(const modbus_driver_interfaces__srv__ModbusRequest_Response__Sequence * lhs, const modbus_driver_interfaces__srv__ModbusRequest_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!modbus_driver_interfaces__srv__ModbusRequest_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
modbus_driver_interfaces__srv__ModbusRequest_Response__Sequence__copy(
  const modbus_driver_interfaces__srv__ModbusRequest_Response__Sequence * input,
  modbus_driver_interfaces__srv__ModbusRequest_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(modbus_driver_interfaces__srv__ModbusRequest_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    modbus_driver_interfaces__srv__ModbusRequest_Response * data =
      (modbus_driver_interfaces__srv__ModbusRequest_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!modbus_driver_interfaces__srv__ModbusRequest_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          modbus_driver_interfaces__srv__ModbusRequest_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!modbus_driver_interfaces__srv__ModbusRequest_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
