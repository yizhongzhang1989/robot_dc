cmake_minimum_required(VERSION 3.10)
project(modbus_driver)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(modbus_driver_interfaces REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  modbus_driver/modbus_manager_node.py
  modbus_driver/modbus_client_tester.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
