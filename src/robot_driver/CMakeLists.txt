cmake_minimum_required(VERSION 3.8)
project(robot_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(PkgConfig REQUIRED)

pkg_check_modules(GPIOD libgpiod)

add_executable(motor_controller src/motor_controller.cpp)
ament_target_dependencies(motor_controller rclcpp geometry_msgs)
target_link_libraries(motor_controller ${GPIOD_LIBRARIES})
target_include_directories(motor_controller PUBLIC ${GPIOD_INCLUDE_DIRS})

install(TARGETS motor_controller
  DESTINATION lib/${PROJECT_NAME})

ament_package()
