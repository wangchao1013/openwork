cmake_minimum_required(VERSION 3.4.1)

project(sg_filter)

add_library(${PROJECT_NAME} STATIC ${CMAKE_CURRENT_SOURCE_DIR}/src/sg_filter.cpp)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_17)