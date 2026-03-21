cmake_minimum_required(VERSION 3.8)
project(my_robot_demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)

add_executable(moveit_PSI_demo src/moveit_PSI_demo.cpp)
ament_target_dependencies(moveit_PSI_demo rclcpp moveit_ros_planning_interface)

add_executable(moveit_MTC_demo src/moveit_MTC_demo.cpp)
ament_target_dependencies(
  moveit_MTC_demo
  rclcpp
  moveit_ros_planning_interface
  moveit_task_constructor_core
)

install(TARGETS
  moveit_PSI_demo
  moveit_MTC_demo
  DESTINATION lib/${PROJECT_NAME}/
)

ament_package()
