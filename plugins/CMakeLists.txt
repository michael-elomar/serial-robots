cmake_minimum_required(VERSION 2.8.3)
project(serial_robots_plugins)

find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})

link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "../bin")

add_library(ur10_joint_controller SHARED src/ur10_joint_controller.cc)
target_link_libraries(ur10_joint_controller ${GAZEBO_LIBRARIES})
