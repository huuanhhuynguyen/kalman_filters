cmake_minimum_required(VERSION 3.15)

set(THIS kalman_filters_cpp)
project(${THIS})

set(CMAKE_CXX_STANDARD 14)

file(GLOB HEADERS include/*.h)
file(GLOB SOURCES src/*.cpp)

include_directories(include)
add_executable(${THIS} ${HEADERS} ${SOURCES})