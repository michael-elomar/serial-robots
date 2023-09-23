cmake_minimum_required(VERSION 3.1)
project(serial_robots_plugins)

# ROS packages

find_package(roscpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(${roscpp_INCLUDE_DIRS})
include_directories(${std_msgs_INCLUDE_DIRS})
include_directories(${geometry_msgs_INCLUDE_DIRS})

find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})

link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "../bin")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "../bin")

add_library(ur10_joint_controller SHARED src/ur10_joint_controller.cc)
target_link_libraries(ur10_joint_controller ${GAZEBO_LIBRARIES})


add_library(ur10_controller SHARED src/ur10_controller.cc)
target_link_libraries(ur10_controller ${GAZEBO_LIBRARIES} ${roscpp_LIBRARIES} ${std_msgs_LIBRARIES} ${geometry_msgs_LIBRARIES})

add_executable(test src/publisher.cc)
target_link_libraries(test ${GAZEBO_LIBRARIES})