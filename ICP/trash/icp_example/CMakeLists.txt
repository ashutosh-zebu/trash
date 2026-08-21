cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
project(icp_example)

find_package(PCL 1.8 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(icp_example main.cpp)
target_link_libraries(icp_example ${PCL_LIBRARIES})
