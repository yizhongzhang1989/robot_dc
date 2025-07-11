// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from modbus_driver_interfaces:msg/ModbusPacket.idl
// generated code does not contain a copyright notice

#ifndef MODBUS_DRIVER_INTERFACES__MSG__DETAIL__MODBUS_PACKET__STRUCT_HPP_
#define MODBUS_DRIVER_INTERFACES__MSG__DETAIL__MODBUS_PACKET__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__modbus_driver_interfaces__msg__ModbusPacket __attribute__((deprecated))
#else
# define DEPRECATED__modbus_driver_interfaces__msg__ModbusPacket __declspec(deprecated)
#endif

namespace modbus_driver_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ModbusPacket_
{
  using Type = ModbusPacket_<ContainerAllocator>;

  explicit ModbusPacket_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->function_code = 0;
      this->slave_id = 0;
      this->address = 0;
      this->count = 0;
    }
  }

  explicit ModbusPacket_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->function_code = 0;
      this->slave_id = 0;
      this->address = 0;
      this->count = 0;
    }
  }

  // field types and members
  using _function_code_type =
    uint8_t;
  _function_code_type function_code;
  using _slave_id_type =
    uint8_t;
  _slave_id_type slave_id;
  using _address_type =
    uint16_t;
  _address_type address;
  using _count_type =
    uint16_t;
  _count_type count;
  using _values_type =
    std::vector<uint16_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint16_t>>;
  _values_type values;

  // setters for named parameter idiom
  Type & set__function_code(
    const uint8_t & _arg)
  {
    this->function_code = _arg;
    return *this;
  }
  Type & set__slave_id(
    const uint8_t & _arg)
  {
    this->slave_id = _arg;
    return *this;
  }
  Type & set__address(
    const uint16_t & _arg)
  {
    this->address = _arg;
    return *this;
  }
  Type & set__count(
    const uint16_t & _arg)
  {
    this->count = _arg;
    return *this;
  }
  Type & set__values(
    const std::vector<uint16_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint16_t>> & _arg)
  {
    this->values = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    modbus_driver_interfaces::msg::ModbusPacket_<ContainerAllocator> *;
  using ConstRawPtr =
    const modbus_driver_interfaces::msg::ModbusPacket_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<modbus_driver_interfaces::msg::ModbusPacket_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<modbus_driver_interfaces::msg::ModbusPacket_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      modbus_driver_interfaces::msg::ModbusPacket_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<modbus_driver_interfaces::msg::ModbusPacket_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      modbus_driver_interfaces::msg::ModbusPacket_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<modbus_driver_interfaces::msg::ModbusPacket_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<modbus_driver_interfaces::msg::ModbusPacket_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<modbus_driver_interfaces::msg::ModbusPacket_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__modbus_driver_interfaces__msg__ModbusPacket
    std::shared_ptr<modbus_driver_interfaces::msg::ModbusPacket_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__modbus_driver_interfaces__msg__ModbusPacket
    std::shared_ptr<modbus_driver_interfaces::msg::ModbusPacket_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ModbusPacket_ & other) const
  {
    if (this->function_code != other.function_code) {
      return false;
    }
    if (this->slave_id != other.slave_id) {
      return false;
    }
    if (this->address != other.address) {
      return false;
    }
    if (this->count != other.count) {
      return false;
    }
    if (this->values != other.values) {
      return false;
    }
    return true;
  }
  bool operator!=(const ModbusPacket_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ModbusPacket_

// alias to use template instance with default allocator
using ModbusPacket =
  modbus_driver_interfaces::msg::ModbusPacket_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace modbus_driver_interfaces

#endif  // MODBUS_DRIVER_INTERFACES__MSG__DETAIL__MODBUS_PACKET__STRUCT_HPP_
