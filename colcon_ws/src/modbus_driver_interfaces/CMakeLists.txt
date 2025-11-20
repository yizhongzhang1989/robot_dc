cmake_minimum_required(VERSION 3.10)
project(modbus_driver_interfaces)

# Suppress CMake policy CMP0148 warning about deprecated FindPythonInterp
if(POLICY CMP0148)
  cmake_policy(SET CMP0148 OLD)
endif()

# Explicitly mark the colcon variable as used to suppress warning
if(DEFINED CATKIN_INSTALL_INTO_PREFIX_ROOT)
  # This variable is passed by colcon for compatibility but not used in ament_cmake
  set(CATKIN_INSTALL_INTO_PREFIX_ROOT ${CATKIN_INSTALL_INTO_PREFIX_ROOT})
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/ModbusRequest.srv"
  "msg/ModbusPacket.msg"
  "msg/MotorSimulationStatus.msg"
)

ament_package()
