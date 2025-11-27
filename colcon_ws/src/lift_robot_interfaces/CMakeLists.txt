cmake_minimum_required(VERSION 3.8)
project(lift_robot_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Declare ROS2 interfaces to generate
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/GotoHeight.action"
  "action/ForceControl.action"
  "action/HybridControl.action"
  "action/ManualMove.action"
  "action/StopMovement.action"
  DEPENDENCIES action_msgs std_msgs
)

ament_package()
