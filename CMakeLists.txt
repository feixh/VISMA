cmake_minimum_required(VERSION 3.5)
project(VISMA)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/CMakeModules)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -O3 -Wall -g -fopenmp -fPIC")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-sign-compare -Wno-unused-function -Wno-unused-variable -Wno-narrowing")
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin )
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib )

# MACROS eigen
add_definitions(-DEIGEN_INITIALIZE_MATRICES_BY_ZERO)

# OpenCV
find_package(OpenCV REQUIRED)
# Eigen
find_package(Eigen3 REQUIRED)
# Google log
find_package(Glog REQUIRED)
# Protobuf
find_package(Protobuf REQUIRED)
PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS ${PROJECT_SOURCE_DIR}/protocols/vlslam.proto)

add_subdirectory(thirdparty/abseil-cpp)
add_subdirectory(thirdparty/jsoncpp)

include_directories(
        ${CMAKE_CURRENT_BINARY_DIR}
        ${PROJECT_SOURCE_DIR}/core
        ${PROJECT_SOURCE_DIR}/include
        ${PROJECT_SOURCE_DIR}/thirdparty/libigl/include
        ${PROJECT_SOURCE_DIR}/thirdparty/jsoncpp/include
        ${PROJECT_SOURCE_DIR}/thirdparty/abseil-cpp

        ${EIGEN3_INCLUDE_DIR}
        ${PROTOBUF_INCLUDE_DIRS}
        ${GLOG_INCLUDE_DIRS}
)
message(STATUS "Found Eigen at ${EIGEN3_INCLUDE_DIR}")

link_directories(
        ${PROJECT_SOURCE_DIR}/lib
        ${PROJECT_SOURCE_DIR}/jsoncpp/lib
        /usr/local/lib
        /usr/lib
)


link_libraries(
        protobuf
        glog
        jsoncpp
        absl::str_format

        opencv_highgui
        opencv_core
        opencv_imgproc
        opencv_video
        opencv_calib3d)

add_library(core SHARED
        core/utils.cpp
        ${PROTO_SRCS})

add_library(visma SHARED
        src/dataloader.cpp
        src/undistorter.cpp
        ${PROTO_SRCS})


add_executable(example_load example/example_load.cpp)
target_link_libraries(example_load visma core)

add_executable(example_dump example/example_dump.cpp)
target_link_libraries(example_dump visma core)

add_executable(undistort_images example/undistort_images.cpp)
target_link_libraries(undistort_images visma core)


set(build_evaluation True)
if (${build_evaluation})
    include_directories(
            ${PROJECT_SOURCE_DIR}/thirdparty/Open3D/include/Open3D)
    link_directories(
            ${PROJECT_SOURCE_DIR}/thirdparty/Open3D/lib
    )
    add_library(visma_eval SHARED
            src/constrained_ICP.cpp
            src/evaluation.cpp
            src/visualization.cpp)

    add_executable(example_evaluate example/example_evaluate.cpp)
    target_link_libraries(example_evaluate visma_eval visma core Open3D)
endif(${build_evaluation})

