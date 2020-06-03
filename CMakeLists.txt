cmake_minimum_required(VERSION 3.16)
project(a_star)

set(CMAKE_CXX_STANDARD 17)

add_executable(a_star main.cpp AStarAlgorithm.h AStarAlgorithm.cpp)
target_link_libraries(a_star)