cmake_minimum_required(VERSION 3.8)
project(human_detection_module)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Găsește pachetele ROS 2
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(ament_index_cpp REQUIRED)

# Găsește OpenCV (necesar pentru cv::dnn)
find_package(OpenCV REQUIRED)

# Crearea executabilului C++
add_executable(detection_node src/detection_node.cpp)

# Includerea librăriilor OpenCV
target_include_directories(detection_node PUBLIC
  ${OpenCV_INCLUDE_DIRS}
)

# Legarea cu pachetele ROS 2
ament_target_dependencies(detection_node
  rclcpp
  sensor_msgs
  geometry_msgs
  cv_bridge
  ament_index_cpp
)

# Linkuirea fizică a OpenCV
target_link_libraries(detection_node ${OpenCV_LIBS})

# ----------------- REGULI DE INSTALARE -----------------
# 1. Instalăm executabilul C++
install(TARGETS
  detection_node
  DESTINATION lib/${PROJECT_NAME}
)

# 2. Instalăm folderul de modele (foarte important pentru a găsi .onnx-ul)
install(DIRECTORY model/
  DESTINATION share/${PROJECT_NAME}/model
)

ament_package()