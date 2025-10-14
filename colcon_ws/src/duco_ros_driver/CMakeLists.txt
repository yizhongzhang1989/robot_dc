cmake_minimum_required(VERSION 3.8)
project(duco_ros_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Include directories for headers
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(duco_msg REQUIRED)
find_package(duco_support REQUIRED)

# Build executables with libDucoCobotAPI.a
add_executable(DucoRobotControl src_backup/DucoRobotControl.cpp)
ament_target_dependencies(DucoRobotControl rclcpp sensor_msgs control_msgs duco_msg)
target_link_libraries(DucoRobotControl ${CMAKE_CURRENT_SOURCE_DIR}/lib/libDucoCobotAPI.a)

add_executable(DucoRobotStatus src_backup/DucoRobotStatusNode.cpp src_backup/DucoRobotStatus.cpp)
ament_target_dependencies(DucoRobotStatus rclcpp sensor_msgs control_msgs std_msgs duco_msg)
target_link_libraries(DucoRobotStatus ${CMAKE_CURRENT_SOURCE_DIR}/lib/libDucoCobotAPI.a)

add_executable(DucoDriver src_backup/DucoDriverNode.cpp src_backup/DucoDriver.cpp)
ament_target_dependencies(DucoDriver rclcpp sensor_msgs trajectory_msgs control_msgs std_msgs duco_msg)
target_link_libraries(DucoDriver ${CMAKE_CURRENT_SOURCE_DIR}/lib/libDucoCobotAPI.a)

add_executable(DucoTrajectoryAction src_backup/DucoTrajectoryAction.cpp)
ament_target_dependencies(DucoTrajectoryAction rclcpp rclcpp_action sensor_msgs trajectory_msgs control_msgs std_msgs duco_msg)
target_link_libraries(DucoTrajectoryAction ${CMAKE_CURRENT_SOURCE_DIR}/lib/libDucoCobotAPI.a)

install(TARGETS DucoRobotControl DucoRobotStatus DucoDriver DucoTrajectoryAction
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Install header files
install(
  DIRECTORY include/
  DESTINATION include
)

# Install library file
install(
  FILES lib/libDucoCobotAPI.a
  DESTINATION lib
)

# Install launch files
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install config files
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
