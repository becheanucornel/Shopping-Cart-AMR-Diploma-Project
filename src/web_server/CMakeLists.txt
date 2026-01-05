cmake_minimum_required(VERSION 3.8)
project(web_server)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

# Include directories
include_directories(
  include
  ${Boost_INCLUDE_DIRS}
)

# Create executable
add_executable(web_server_node
  src/web_server_node.cpp
)

ament_target_dependencies(web_server_node
  rclcpp
  rclcpp_components
  ament_index_cpp
)

target_link_libraries(web_server_node
  ${Boost_LIBRARIES}
)

# Install executable
install(TARGETS
  web_server_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install resource files
install(DIRECTORY
  resource/
  DESTINATION share/${PROJECT_NAME}/resource
)

# Install include directory
install(DIRECTORY
  include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
