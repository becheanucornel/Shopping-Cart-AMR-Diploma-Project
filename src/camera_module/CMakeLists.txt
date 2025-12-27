cmake_minimum_required(VERSION 3.8)
project(camera_module)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(image_transport REQUIRED)

add_executable(video_stream_publisher src/camera_stream_publisher.cpp)
ament_target_dependencies(video_stream_publisher rclcpp sensor_msgs cv_bridge OpenCV)

install(TARGETS
  video_stream_publisher
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
