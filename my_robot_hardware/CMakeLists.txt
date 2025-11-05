cmake_minimum_required(VERSION 3.8)
project(my_robot_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find required ROS 2 packages
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(controller_interface REQUIRED)
find_package(controller_manager REQUIRED)

# Include headers
include_directories(include)

# Build the hardware plugin library
add_library(my_robot_system_hardware SHARED
  src/my_robot_system.cpp
)

# Link dependencies
ament_target_dependencies(my_robot_system_hardware
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  controller_interface
  controller_manager
)

# Export plugin description
pluginlib_export_plugin_description_file(hardware_interface plugin_description.xml)

# Install library
install(TARGETS my_robot_system_hardware
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)

# Install headers
install(DIRECTORY include/
        DESTINATION include)

# Install plugin description
install(FILES plugin_description.xml
        DESTINATION share/${PROJECT_NAME})

# Install launch, URDF, config files
install(DIRECTORY launch urdf config
        DESTINATION share/${PROJECT_NAME})

# Declare package
ament_package()
