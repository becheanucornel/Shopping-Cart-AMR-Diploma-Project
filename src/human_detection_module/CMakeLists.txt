cmake_minimum_required(VERSION 3.8)
project(human_detection_module)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake   REQUIRED)
find_package(rclcpp        REQUIRED)
find_package(sensor_msgs   REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs      REQUIRED)
find_package(cv_bridge     REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(OpenCV REQUIRED)─
find_package(CUDA REQUIRED)

set(CUDA_INCLUDE_DIRS /usr/local/cuda/include)
set(CUDA_LIBRARIES    /usr/local/cuda/lib64/libcudart.so)
set(TRT_INCLUDE_DIR /usr/include/aarch64-linux-gnu)
set(TRT_LIB_DIR     /usr/lib/aarch64-linux-gnu)

find_library(TRT_NVINFER   nvinfer        HINTS ${TRT_LIB_DIR} REQUIRED)
find_library(TRT_NVONNX    nvonnxparser   HINTS ${TRT_LIB_DIR} REQUIRED)
find_library(TRT_NVINFER_PLUGIN nvinfer_plugin HINTS ${TRT_LIB_DIR})

add_executable(detection_node src/detection_node.cpp)

target_include_directories(detection_node PUBLIC
  ${OpenCV_INCLUDE_DIRS}
  ${CUDA_INCLUDE_DIRS}
  ${TRT_INCLUDE_DIR}
)

ament_target_dependencies(detection_node
  rclcpp
  sensor_msgs
  geometry_msgs
  std_srvs
  cv_bridge
  ament_index_cpp
)

target_link_libraries(detection_node
  ${OpenCV_LIBS}
  ${CUDA_LIBRARIES}
  ${TRT_NVINFER}
  ${TRT_NVONNX}
)

install(TARGETS detection_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY model/
  DESTINATION share/${PROJECT_NAME}/model
)

ament_package()