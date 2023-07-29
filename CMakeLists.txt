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

file(DOWNLOAD https://doc-0g-6k-docs.googleusercontent.com/docs/securesc/bspganatg096vrniqtpa67du2027u0qu/mr9mck2qopnotipg3q554fce738bpqjq/1690529550000/18116677926076642180/01627000481668383189/1ttdVtlDiPun13EQCB4Nyls3Q8w5aXg1i?e=download&ax=AGtFMPXi8MHK9FGxhvWjmcqYYM9OHg8AhDus4Xjf297C2f0BOOqb17hu-10XeGkicqhEaLaPd6Hm91KeDyXhivPKhpyzD3LVlVXFqBFtJUCGk-HlGk5jC4Jo7a0mgW8eWg_xWJX2Iq554GCUu8ZisSZ_YwHNuqOz9HTQpNTa3TfE9bHVMPKx_7168TDrQeShdtiakGEgVszliLZT7cEgvovLT0e4RzRnXcj-kdfPe_FMjelAIpnNP2KBkamEdIBaOKrE_WFNAjVMtNtpu2KS0-66eHaTvHoVp3KZN91x4rkVnUOOrY3-6PEDCuQorelOSsyPD9Ddp9VbJrnoGABo3vjunsR8ewZzCQrI9iCV_0q1SYq03KA7QjgmV3p_8U91Ezu80_naED_lznX4fuQYjzZNJjcoUrglcFYTxgFP-HOzGI1W-zV6Ke7uuDvFq3gkn3_a2XMXRHq_efq_fqyYVzVPH6kH_WsgBitPf94gMdSqgCcsD3VhjQGkbWxkXqSzZrHPAiVZkHybAJRe8Qi3cTgLHW-vRpmSJ9oAyUEVydoDyzH4HIPWb8io8kVYbc4aJhJJOS7Z9creoOAatANM_HHDWlFEztgtAVyldPaRDROoneV-5faXhr0SkNwdxf8kuJxP9eqbPqe2Ghi_-j40Z8bkSp6P7y45GadS5asyLBWkucLxkcp-6KGCUNPmPiFEfgoD_eKjqWOq7rmPMr12OR1uO_zGEhrTASAQWwfj13t3wYPEipsdTkEGuxRsWvwX9XKV_jcTSgPWmMQ9TZuyB5oqZCiJr1poSqK6vNtRGVzWsF6cdm6i_x5eQ1dtJ47tepTT1GRPRXm4WiMEcRxJtxhHv5IrlSU7LwYCjQvR2Fuzivsf3zbc0W-J3FV_wJG5M9VNZBrN-dZFQjPmMbELTuLjAJ3GDWq6WQlVDvKgl2dqna2qeIATi__TgVSQXGY_0rars9WCkCnD3w&uuid=4f26ee8d-f71f-4742-b6fa-71eb30718a57&authuser=1&nonce=jadboi7fljjiu&user=01627000481668383189&hash=gm34bqp4eg36322glb70ukosdb5ehq2q lightNet-TRT/configs/lightNet-BDD100K-det-semaseg-1280x960.weights)

file(GLOB_RECURSE sources lightNet-TRT/modules/*.hpp lightNet-TRT/modules/*.cpp lightNet-TRT/modules/*.h lightNet-TRT/modules/*.cu lightNet-TRT/extra/*.h lightNet-TRT/module/preprocess.cu)
add_library(detector SHARED ${sources})
target_include_directories(detector PRIVATE lightNet-TRT/extra/ lightNet-TRT/modules/ ${OpenCV_INCLUDE_DIRS} ${CUDA_TOOLKIT_ROOT_DIR}/include)
target_link_libraries(detector nvinfer nvinfer_plugin nvparsers gflags "stdc++fs")

add_library(${PROJECT_NAME} SHARED tensorrt_lightnet.cpp)
target_include_directories(${PROJECT_NAME} PRIVATE lightNet-TRT/extra/ lightNet-TRT/modules/ ${OpenCV_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} detector)

add_library(${PROJECT_NAME}_node SHARED tensorrt_lightnet_node.cpp)
target_include_directories(${PROJECT_NAME}_node PRIVATE lightNet-TRT/extra/ lightNet-TRT/modules/ ${OpenCV_INCLUDE_DIRS} ${rclcpp_INCLUDE_DIRS} ${sensor_msgs_INCLUDE_DIRS} ${rclcpp_components_INCLUDE_DIRS})
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
    lightNet-TRT/configs
    launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()