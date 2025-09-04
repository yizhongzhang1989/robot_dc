cmake_minimum_required(VERSION 3.16)
project(duco_hardware)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

add_library(
  duco_hardware
  SHARED
  src/duco_hardware_interface.cpp
)
target_compile_features(duco_hardware PUBLIC cxx_std_17)
target_include_directories(duco_hardware PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include/duco_hardware>
)

# Link the DUCO RPC library (same as used by duco_ros_driver)  
target_link_libraries(duco_hardware PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../duco_ros_driver/lib/libDucoCobotAPI.a)

ament_target_dependencies(
  duco_hardware PUBLIC
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# Add DUCO RPC dependency - header only library
target_include_directories(duco_hardware PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/../duco_ros_driver/include
)

pluginlib_export_plugin_description_file(hardware_interface duco_hardware.xml)

install(
  DIRECTORY include/
  DESTINATION include/duco_hardware
)
install(
  TARGETS duco_hardware
  EXPORT export_duco_hardware
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_duco_hardware HAS_LIBRARY_TARGET)
ament_export_dependencies(hardware_interface pluginlib rclcpp rclcpp_lifecycle)
ament_package()
