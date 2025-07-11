// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from modbus_driver_interfaces:msg/MotorSimulationStatus.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "modbus_driver_interfaces/msg/detail/motor_simulation_status__rosidl_typesupport_introspection_c.h"
#include "modbus_driver_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "modbus_driver_interfaces/msg/detail/motor_simulation_status__functions.h"
#include "modbus_driver_interfaces/msg/detail/motor_simulation_status__struct.h"


// Include directives for member types
// Member `last_command`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void modbus_driver_interfaces__msg__MotorSimulationStatus__rosidl_typesupport_introspection_c__MotorSimulationStatus_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  modbus_driver_interfaces__msg__MotorSimulationStatus__init(message_memory);
}

void modbus_driver_interfaces__msg__MotorSimulationStatus__rosidl_typesupport_introspection_c__MotorSimulationStatus_fini_function(void * message_memory)
{
  modbus_driver_interfaces__msg__MotorSimulationStatus__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember modbus_driver_interfaces__msg__MotorSimulationStatus__rosidl_typesupport_introspection_c__MotorSimulationStatus_message_member_array[7] = {
  {
    "motor_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(modbus_driver_interfaces__msg__MotorSimulationStatus, motor_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "current_position",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(modbus_driver_interfaces__msg__MotorSimulationStatus, current_position),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "target_position",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(modbus_driver_interfaces__msg__MotorSimulationStatus, target_position),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "velocity",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(modbus_driver_interfaces__msg__MotorSimulationStatus, velocity),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "motion_mode",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(modbus_driver_interfaces__msg__MotorSimulationStatus, motion_mode),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "moving",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(modbus_driver_interfaces__msg__MotorSimulationStatus, moving),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "last_command",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(modbus_driver_interfaces__msg__MotorSimulationStatus, last_command),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers modbus_driver_interfaces__msg__MotorSimulationStatus__rosidl_typesupport_introspection_c__MotorSimulationStatus_message_members = {
  "modbus_driver_interfaces__msg",  // message namespace
  "MotorSimulationStatus",  // message name
  7,  // number of fields
  sizeof(modbus_driver_interfaces__msg__MotorSimulationStatus),
  modbus_driver_interfaces__msg__MotorSimulationStatus__rosidl_typesupport_introspection_c__MotorSimulationStatus_message_member_array,  // message members
  modbus_driver_interfaces__msg__MotorSimulationStatus__rosidl_typesupport_introspection_c__MotorSimulationStatus_init_function,  // function to initialize message memory (memory has to be allocated)
  modbus_driver_interfaces__msg__MotorSimulationStatus__rosidl_typesupport_introspection_c__MotorSimulationStatus_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t modbus_driver_interfaces__msg__MotorSimulationStatus__rosidl_typesupport_introspection_c__MotorSimulationStatus_message_type_support_handle = {
  0,
  &modbus_driver_interfaces__msg__MotorSimulationStatus__rosidl_typesupport_introspection_c__MotorSimulationStatus_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_modbus_driver_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, modbus_driver_interfaces, msg, MotorSimulationStatus)() {
  if (!modbus_driver_interfaces__msg__MotorSimulationStatus__rosidl_typesupport_introspection_c__MotorSimulationStatus_message_type_support_handle.typesupport_identifier) {
    modbus_driver_interfaces__msg__MotorSimulationStatus__rosidl_typesupport_introspection_c__MotorSimulationStatus_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &modbus_driver_interfaces__msg__MotorSimulationStatus__rosidl_typesupport_introspection_c__MotorSimulationStatus_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
