cmake_minimum_required(VERSION 3.8)
project(duco_ros_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Include directories for headers
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

# Find dependencies
find_package(ament_cmake REQUIRED)

# This package only provides headers and library files for duco_hardware
# No executables are built since they depend on deleted duco_msg package

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

ament_package()
