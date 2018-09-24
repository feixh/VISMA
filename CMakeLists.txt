cmake_minimum_required(VERSION 3.8)
project(VISMA)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/CMakeModules)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3 -Wall -g -fopenmp")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-sign-compare -Wno-unused-function -Wno-unused-variable")
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin )
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib )

# MACROS eigen
add_definitions(-DEIGEN_INITIALIZE_MATRICES_BY_ZERO)

# OpenCV
find_package(OpenCV REQUIRED)
# Eigen
find_package(Eigen3 REQUIRED)
# Protobuf
find_package(Protobuf REQUIRED)
PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS ${PROJECT_SOURCE_DIR}/protocols/vlslam.proto)

# facebook folly
find_package(Folly REQUIRED)

include_directories(
        ${CMAKE_CURRENT_BINARY_DIR}
        ${PROJECT_SOURCE_DIR}/include
        ${PROJECT_SOURCE_DIR}/thirdparty/Sophus/include

        ${OpenCV_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
        ${PROTOBUF_INCLUDE_DIRS}
        ${FOLLY_INCLUDE_DIR}
)
message(STATUS "Found Eigen at ${EIGEN3_INCLUDE_DIR}")

link_directories(
        ${PROJECT_SOURCE_DIR}/lib
        /opt/ros/kinetic/lib    # In case OpenCV libs are installed in a non-standard location, specify the location here.
)


set(OpenCV_MINIMAL
        opencv_highgui3
        opencv_core3
        opencv_imgcodecs3
        opencv_imgproc3
        opencv_videoio3
        opencv_calib3d3)


link_libraries(
        protobuf
        ${OpenCV_MINIMAL}
        ${FOLLY_LIBRARY}
        double-conversion   # for folly
)

add_library(visma SHARED
        src/dataset_loaders.cpp
        src/io_utils.cpp
        src/common/utils.cpp
        ${PROTO_SRCS})


add_executable(example_load example/example_load.cpp)
target_link_libraries(example_load visma)
