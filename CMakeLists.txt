# CMake version needs to be at least 3.3 for rclcpp
cmake_minimum_required(VERSION 3.3)
project(tensorrt_lightnet)
enable_language(CUDA)

find_package(OpenMP REQUIRED)
set(CMAKE_CXX_COMPILER "/usr/bin/g++")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -O3 -Wno-write-strings ${OpenMP_CXX_FLAGS} -fopenmp")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-rpath -Wl,$ORIGIN")

include_directories(/usr/local/cuda/include)
find_package(CUDA REQUIRED)

find_package(OpenCV REQUIRED)

find_package(image_transport REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(rclcpp_components REQUIRED)

file(
  DOWNLOAD
  "https://drive.google.com/uc?export=download&id=1ttdVtlDiPun13EQCB4Nyls3Q8w5aXg1i"
  "${CMAKE_SOURCE_DIR}/lib/configs/lightNet-BDD100K-det-semaseg-1280x960.weights"
  TIMEOUT 600
)

file(GLOB_RECURSE sources lib/modules/*.hpp lib/modules/*.cpp lib/modules/*.h lib/modules/*.cu lib/extra/*.h lib/module/preprocess.cu)
add_library(detector SHARED ${sources})
target_include_directories(detector PRIVATE lib/extra/ lib/modules/ ${OpenCV_INCLUDE_DIRS} ${CUDA_TOOLKIT_ROOT_DIR}/include)
target_link_libraries(detector nvinfer nvinfer_plugin nvparsers gflags "stdc++fs" ${OpenCV_LIBS})

add_library(${PROJECT_NAME} SHARED src/tensorrt_lightnet.cpp)
target_include_directories(${PROJECT_NAME} PRIVATE lib/extra/ lib/modules/ ${OpenCV_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} detector)

add_library(${PROJECT_NAME}_node SHARED src/tensorrt_lightnet_node.cpp)
target_include_directories(${PROJECT_NAME}_node PRIVATE lib/extra/ lib/modules/ ${OpenCV_INCLUDE_DIRS} ${rclcpp_INCLUDE_DIRS} ${sensor_msgs_INCLUDE_DIRS} ${rclcpp_components_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME} image_transport::image_transport cv_bridge::cv_bridge)

rclcpp_components_register_nodes(${PROJECT_NAME}_node
  PLUGIN "tensorrt_lightnet::TrtLightNetNode"
  EXECUTABLE ${PROJECT_NAME}_node_exe
)

install(
  TARGETS
    detector
    ${PROJECT_NAME}
    ${PROJECT_NAME}_node
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY
    lib/configs
    launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()