cmake_minimum_required(VERSION 3.10)
project(PclObstacleDetection)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)

find_package(PCL 1.8 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(pcl_obstacle_detection src/main.cpp)
target_link_libraries(pcl_obstacle_detection ${PCL_LIBRARIES})

# Recommended: Generate compile_commands.json for tooling (e.g., clangd)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
