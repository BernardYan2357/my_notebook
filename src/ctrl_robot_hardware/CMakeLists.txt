cmake_minimum_required(VERSION 3.8)
project(ctrl_robot_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)

# include headers
include_directories(include)

add_library(ctrl_robot_hardware_plugin SHARED
  src/mobile_base_hardware_interface.cpp
  src/example_motor_driver.cpp
)

target_include_directories(ctrl_robot_hardware_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(ctrl_robot_hardware_plugin
  hardware_interface
  pluginlib
  rclcpp
)

pluginlib_export_plugin_description_file(hardware_interface ctrl_robot_hardware_plugin.xml)

install(TARGETS ctrl_robot_hardware_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES ctrl_robot_hardware_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
