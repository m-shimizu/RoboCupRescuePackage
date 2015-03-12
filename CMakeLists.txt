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
# Build the USARGazebo plugin
add_library(USARGazebo SHARED USARGazebo.cc)
target_link_libraries(USARGazebo ${GAZEBO_LIBRARIES})

################################################
# Build the example a set of USARGazebo-WorldPlugin(Subscriber) and usarsim_pub program
add_library(USARGazeboW SHARED USARGazebo-WorldPluginSample.cc)
target_link_libraries(USARGazeboW ${GAZEBO_LIBRARIES})
add_executable(usarsim_pub usarsim_pub.cc)
target_link_libraries(usarsim_pub ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})

add_executable (robot_teleop robot_teleop.cc)
target_link_libraries(robot_teleop ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})
# target_link_libraries(robot_teleop ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} ${SDF_LIBRARIES})

