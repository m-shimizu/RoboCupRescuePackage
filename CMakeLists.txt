cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

################################################
# Find Gazebo
find_package(GAZEBO)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

################################################
# Build the USRARaz plugin
add_library(USARGaz SHARED USARGaz1.cc)
target_link_libraries(USARGaz ${GAZEBO_LIBRARIES})

################################################
# Build the example usarsim_pub program
add_executable(usarsim_pub usarsim_pub.cc)
target_link_libraries(usarsim_pub ${GAZEBO_LIBRARIES})
