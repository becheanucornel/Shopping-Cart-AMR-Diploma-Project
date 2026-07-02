cmake_minimum_required(VERSION 3.8)
project(scan_merger_module)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(scan_merger_node src/scan_merger_node.cpp)
ament_target_dependencies(scan_merger_node rclcpp sensor_msgs geometry_msgs tf2 tf2_ros)

install(TARGETS
  scan_merger_node
  DESTINATION lib/${PROJECT_NAME}
  )

ament_package()
