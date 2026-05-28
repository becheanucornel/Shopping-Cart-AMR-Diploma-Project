cmake_minimum_required(VERSION 3.16)
project(web_server)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake       REQUIRED)
find_package(rclcpp            REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_action     REQUIRED)
find_package(geometry_msgs     REQUIRED)
find_package(nav2_msgs         REQUIRED)
find_package(nav_msgs          REQUIRED)
find_package(std_msgs          REQUIRED)
find_package(sensor_msgs       REQUIRED) 
find_package(cv_bridge         REQUIRED)   
find_package(ament_index_cpp   REQUIRED)
find_package(tf2_ros           REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(OpenCV            REQUIRED)
find_package(Boost             REQUIRED)

add_library(web_server_lib SHARED
  src/web_server_node.cpp
)

target_include_directories(web_server_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
)

ament_target_dependencies(web_server_lib
  rclcpp
  rclcpp_components
  rclcpp_action
  geometry_msgs
  nav2_msgs
  nav_msgs
  std_msgs
  sensor_msgs
  cv_bridge
  ament_index_cpp
  tf2_ros
  tf2_geometry_msgs
  Boost
)

target_link_libraries(web_server_lib
  ${OpenCV_LIBS}
)

rclcpp_components_register_node(web_server_lib
  PLUGIN "web_server::WebServerNode"
  EXECUTABLE "web_server_node"
)

install(TARGETS web_server_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/web_server
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY resource/
  DESTINATION share/web_server/resource
)

ament_package()
