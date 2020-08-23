cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(model_pose CXX)

find_package(gazebo REQUIRED)

include_directories(${GAZEBO_INCLUDE_DIRS})

link_directories(${GAZEBO_LIBRARY_DIRS})

list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_library(model_pose SHARED main.cpp)

target_link_libraries(model_pose ${GAZEBO_LIBRARIES})