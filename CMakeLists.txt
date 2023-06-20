cmake_minimum_required(VERSION 3.10)
project(g2oFit)

set(CMAKE_CXX_STANDARD 11)

find_package(Eigen3 REQUIRED)
find_package(g2o REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIRS})
include_directories(${G2O_INCLUDE_DIRS})

add_executable(g2o_fit g2o_fit.cpp)
target_link_libraries(g2o_fit g2o_core g2o_stuff)
