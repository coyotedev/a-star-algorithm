cmake_minimum_required(VERSION 3.16)
project(a_star)

set(CMAKE_CXX_STANDARD 14)

add_executable(a_star main.cpp AStarAlgorithm.hpp)
target_link_libraries(a_star)