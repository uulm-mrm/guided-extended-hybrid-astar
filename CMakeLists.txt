cmake_minimum_required(VERSION 3.18)

project(sandbox)

# set build type here
set(CMAKE_BUILD_TYPE Release)

set(SANDBOX_PATH ${PROJECT_SOURCE_DIR})

add_subdirectory(src/freespace_planner/library)
add_subdirectory(src/freespace_planner/ros2)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
