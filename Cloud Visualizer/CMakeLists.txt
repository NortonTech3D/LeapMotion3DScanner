cmake_minimum_required(VERSION 3.16)

project(cloud_viewer CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(PCL 1.7 REQUIRED COMPONENTS common io visualization)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(cloud_viewer cloud_viewer.cpp)
target_link_libraries(cloud_viewer PRIVATE ${PCL_LIBRARIES})