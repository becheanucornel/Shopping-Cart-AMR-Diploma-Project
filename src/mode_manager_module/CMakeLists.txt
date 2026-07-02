cmake_minimum_required(VERSION 3.8)
project(mode_manager_module)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(mode_manager_interfaces REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

add_executable(mode_manager_node src/mode_manager.cpp)
ament_target_dependencies(mode_manager_node rclcpp rclcpp_action std_msgs geometry_msgs nav2_msgs nav_msgs mode_manager_interfaces)

install(TARGETS 
  mode_manager_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
