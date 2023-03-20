cmake_minimum_required(VERSION 3.16)
file(GLOB SRCS "./src/*.cpp")
project(lib-mapf)

add_library(lib-mapf STATIC ${SRCS})
# Set MAPDIR to the directory containing the map files
add_definitions(-D_MAPDIR_= "${CMAKE_CURRENT_LIST_DIR}/../pibt_test/map/")

target_compile_options(lib-mapf PUBLIC -O3 -Wall -mtune=native -march=native)
target_compile_features(lib-mapf PUBLIC cxx_std_17)
target_include_directories(lib-mapf INTERFACE ./include)

add_subdirectory(../third_party/gridmap/graph ./graph)
target_link_libraries(lib-mapf lib-graph)