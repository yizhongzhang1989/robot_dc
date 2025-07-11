// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from modbus_driver_interfaces:srv/ModbusRequest.idl
// generated code does not contain a copyright notice

#ifndef MODBUS_DRIVER_INTERFACES__SRV__DETAIL__MODBUS_REQUEST__STRUCT_HPP_
#define MODBUS_DRIVER_INTERFACES__SRV__DETAIL__MODBUS_REQUEST__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__modbus_driver_interfaces__srv__ModbusRequest_Request __attribute__((deprecated))
#else
# define DEPRECATED__modbus_driver_interfaces__srv__ModbusRequest_Request __declspec(deprecated)
#endif

namespace modbus_driver_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct ModbusRequest_Request_
{
  using Type = ModbusRequest_Request_<ContainerAllocator>;

  explicit ModbusRequest_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->function_code = 0;
      this->slave_id = 0;
      this->address = 0;
      this->count = 0;
      this->seq_id = 0;
    }
  }

  explicit ModbusRequest_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->function_code = 0;
      this->slave_id = 0;
      this->address = 0;
      this->count = 0;
      this->seq_id = 0;
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
  using _seq_id_type =
    uint16_t;
  _seq_id_type seq_id;

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
  Type & set__seq_id(
    const uint16_t & _arg)
  {
    this->seq_id = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    modbus_driver_interfaces::srv::ModbusRequest_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const modbus_driver_interfaces::srv::ModbusRequest_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<modbus_driver_interfaces::srv::ModbusRequest_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<modbus_driver_interfaces::srv::ModbusRequest_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      modbus_driver_interfaces::srv::ModbusRequest_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<modbus_driver_interfaces::srv::ModbusRequest_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      modbus_driver_interfaces::srv::ModbusRequest_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<modbus_driver_interfaces::srv::ModbusRequest_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<modbus_driver_interfaces::srv::ModbusRequest_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<modbus_driver_interfaces::srv::ModbusRequest_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__modbus_driver_interfaces__srv__ModbusRequest_Request
    std::shared_ptr<modbus_driver_interfaces::srv::ModbusRequest_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__modbus_driver_interfaces__srv__ModbusRequest_Request
    std::shared_ptr<modbus_driver_interfaces::srv::ModbusRequest_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ModbusRequest_Request_ & other) const
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
    if (this->seq_id != other.seq_id) {
      return false;
    }
    return true;
  }
  bool operator!=(const ModbusRequest_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ModbusRequest_Request_

// alias to use template instance with default allocator
using ModbusRequest_Request =
  modbus_driver_interfaces::srv::ModbusRequest_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace modbus_driver_interfaces


#ifndef _WIN32
# define DEPRECATED__modbus_driver_interfaces__srv__ModbusRequest_Response __attribute__((deprecated))
#else
# define DEPRECATED__modbus_driver_interfaces__srv__ModbusRequest_Response __declspec(deprecated)
#endif

namespace modbus_driver_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct ModbusRequest_Response_
{
  using Type = ModbusRequest_Response_<ContainerAllocator>;

  explicit ModbusRequest_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->ack = 0;
    }
  }

  explicit ModbusRequest_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->ack = 0;
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;
  using _response_type =
    std::vector<uint16_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint16_t>>;
  _response_type response;
  using _ack_type =
    uint16_t;
  _ack_type ack;

  // setters for named parameter idiom
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }
  Type & set__response(
    const std::vector<uint16_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint16_t>> & _arg)
  {
    this->response = _arg;
    return *this;
  }
  Type & set__ack(
    const uint16_t & _arg)
  {
    this->ack = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    modbus_driver_interfaces::srv::ModbusRequest_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const modbus_driver_interfaces::srv::ModbusRequest_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<modbus_driver_interfaces::srv::ModbusRequest_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<modbus_driver_interfaces::srv::ModbusRequest_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      modbus_driver_interfaces::srv::ModbusRequest_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<modbus_driver_interfaces::srv::ModbusRequest_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      modbus_driver_interfaces::srv::ModbusRequest_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<modbus_driver_interfaces::srv::ModbusRequest_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<modbus_driver_interfaces::srv::ModbusRequest_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<modbus_driver_interfaces::srv::ModbusRequest_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__modbus_driver_interfaces__srv__ModbusRequest_Response
    std::shared_ptr<modbus_driver_interfaces::srv::ModbusRequest_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__modbus_driver_interfaces__srv__ModbusRequest_Response
    std::shared_ptr<modbus_driver_interfaces::srv::ModbusRequest_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ModbusRequest_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    if (this->response != other.response) {
      return false;
    }
    if (this->ack != other.ack) {
      return false;
    }
    return true;
  }
  bool operator!=(const ModbusRequest_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ModbusRequest_Response_

// alias to use template instance with default allocator
using ModbusRequest_Response =
  modbus_driver_interfaces::srv::ModbusRequest_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace modbus_driver_interfaces

namespace modbus_driver_interfaces
{

namespace srv
{

struct ModbusRequest
{
  using Request = modbus_driver_interfaces::srv::ModbusRequest_Request;
  using Response = modbus_driver_interfaces::srv::ModbusRequest_Response;
};

}  // namespace srv

}  // namespace modbus_driver_interfaces

#endif  // MODBUS_DRIVER_INTERFACES__SRV__DETAIL__MODBUS_REQUEST__STRUCT_HPP_
