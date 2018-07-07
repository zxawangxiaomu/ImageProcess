cmake_minimum_required(VERSION 2.8.3)
project(ImageProcess)

add_compile_options(-std=c++11)

find_package(Eigen3 REQUIRED)
include_directories(
    include
    ${EIGEN3_INCLUDE_DIRS}
)

enable_testing()
add_subdirectory(src/CameraModel)
add_subdirectory(example)