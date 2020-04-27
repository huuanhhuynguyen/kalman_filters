cmake_minimum_required(VERSION 3.10)

set(THIS kalman_filters_cpp)

project(${THIS})
set(CMAKE_CXX_STANDARD 14)

include_directories(include)
include_directories(Eigen)
include_directories(matplotlib-cpp)

set(SOURCES src/main.cpp
            src/sample.cpp
            src/kf/EKF.cpp
            src/model/nonlinear.cpp)
add_executable(${THIS} ${SOURCES} )

find_package(Python2 COMPONENTS Development)
target_include_directories(${THIS} PRIVATE ${Python2_INCLUDE_DIRS})
target_link_libraries(${THIS} Python2::Python)