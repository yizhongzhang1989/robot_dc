cmake_minimum_required(VERSION 3.8)
project(robot_status_redis)

# Suppress CMake policy CMP0148 warning about deprecated FindPythonInterp
if(POLICY CMP0148)
  cmake_policy(SET CMP0148 OLD)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(robot_status REQUIRED)

# Install Python module files to the rosidl generated Python package location
# This ensures they're in the same location as the service interfaces
install(FILES
  robot_status_redis/__init__.py
  robot_status_redis/client_utils.py
  robot_status_redis/redis_backend.py
  robot_status_redis/web_dashboard.py
  DESTINATION local/lib/python3.10/dist-packages/robot_status_redis
)

# Install web dashboard templates and static files
install(DIRECTORY
  robot_status_redis/templates
  robot_status_redis/static
  DESTINATION local/lib/python3.10/dist-packages/robot_status_redis
)

# Set PYTHONPATH for the package
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/robot_status_redis.dsv.in")

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install examples
install(DIRECTORY examples
  DESTINATION share/${PROJECT_NAME}
)

# Install Python executables (make them executable)
install(PROGRAMS
  robot_status_redis/robot_status_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
