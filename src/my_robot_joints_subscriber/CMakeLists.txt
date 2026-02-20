cmake_minimum_required(VERSION 3.8)
project(my_robot_joints_subscriber)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(joints_subscriber src/joints_subscriber.cpp)
ament_target_dependencies(joints_subscriber rclcpp sensor_msgs)

install(TARGETS
  joints_subscriber
  DESTINATION lib/${PROJECT_NAME}/
)

ament_package()
