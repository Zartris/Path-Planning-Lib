cmake_minimum_required(VERSION 3.4...3.18)
project(path_planning)
add_library(AStar STATIC source/AStar.cpp)
set_target_properties(AStar PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_subdirectory(thirdparty/pybind11)
pybind11_add_module(path_planning pybinder.cpp)
target_link_libraries(path_planning PRIVATE AStar)
