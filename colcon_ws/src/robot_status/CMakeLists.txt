cmake_minimum_required(VERSION 3.8)
project(robot_status)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclpy REQUIRED)

# Generate services
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetStatus.srv"
  "srv/GetStatus.srv"
  "srv/ListStatus.srv"
  DEPENDENCIES std_msgs
)

# Install Python modules manually
install(DIRECTORY robot_status
  DESTINATION lib/python3.10/site-packages
  PATTERN "__pycache__" EXCLUDE
)

# Set PYTHONPATH for the package
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/robot_status.dsv.in")

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
  robot_status/robot_status_node.py
  robot_status/web_dashboard.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
