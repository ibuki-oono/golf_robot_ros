cmake_minimum_required(VERSION 3.8)
project(my_robot_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(serial REQUIRED)

add_library(my_robot_system_hardware SHARED
  src/my_robot_system.cpp
)

ament_target_dependencies(my_robot_system_hardware
  hardware_interface
  pluginlib
  rclcpp_lifecycle
  serial
)

pluginlib_export_plugin_description_file(hardware_interface plugin_description.xml)

install(TARGETS my_robot_system_hardware
        LIBRARY DESTINATION lib)
install(DIRECTORY include/
        DESTINATION include)
install(FILES plugin_description.xml
        DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch config
        DESTINATION share/${PROJECT_NAME})

ament_package()

