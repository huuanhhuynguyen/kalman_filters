cmake_minimum_required(VERSION 3.10)

set(THIS kalman_filters_cpp)

project(${THIS})
set(CMAKE_CXX_STANDARD 14)

include_directories(src)
include_directories(Eigen)

set(SOURCES src/main.cpp
            src/sample.cpp
            src/kf/EKF.cpp
            src/kf/UKF.cpp
            src/model/nonlinear.cpp
            src/sigma_points.cpp)
add_executable(${THIS} ${SOURCES})

# The following is required for matplotlib-cpp
include_directories(matplotlib-cpp)
find_package(PythonLibs 2.7)
if(NOT PythonLibs_FOUND)
    message(WARNING "PythonLibs 2.7 not found. Make sure you have installed python 2.7.
       If that doesn't solve the problem, consult
       https://github.com/lava/matplotlib-cpp")
endif()
target_include_directories(${THIS} PRIVATE ${PYTHON_INCLUDE_DIRS})
target_link_libraries(${THIS} ${PYTHON_LIBRARIES})