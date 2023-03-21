# Specify minimum required CMake version
cmake_minimum_required(VERSION 3.16...3.18)

# Collect all source files in src directory
file(GLOB SRCS "./src/*.cpp")

# Set project name
project(lib-mapf)

# Create shared library from source files
add_library(lib-mapf SHARED ${SRCS})

# Add preprocessor definition for map directory path
add_definitions(-D_MAPDIR_= "${CMAKE_CURRENT_LIST_DIR}/../pibt_test/map/")

# Set compilation options for lib-mapf
target_compile_options(lib-mapf PUBLIC -O3 -Wall -mtune=native -march=native)

# Specify C++17 support for lib-mapf
target_compile_features(lib-mapf PUBLIC cxx_std_17)

# Set include directory for lib-mapf
target_include_directories(lib-mapf INTERFACE ./include)

# Link lib-mapf with lib-graph
target_link_libraries(lib-mapf PUBLIC lib-graph)


#target_include_directories(lib-mapf PUBLIC ../third_party/gridmap/graph/include)
#add_subdirectory(../third_party/gridmap/graph ./graph)
#target_link_libraries(lib-mapf lib-graph)
#