cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

################################################
# Find System files
find_package(Boost REQUIRED COMPONENTS system)
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})
# Find Gazebo
find_package(GAZEBO)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

################################################
# Build the USARGaz plugin
add_library(USARGaz SHARED USARGaz1.cc)
target_link_libraries(USARGaz ${GAZEBO_LIBRARIES})

################################################
# Build the example a set of USARGaz-WorldPlugin(Subscriber) and usarsim_pub program
add_library(USARGazW SHARED USARGaz-WorldPluginSample.cc)
target_link_libraries(USARGazW ${GAZEBO_LIBRARIES})
add_executable(usarsim_pub usarsim_pub.cc)
target_link_libraries(usarsim_pub ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})

