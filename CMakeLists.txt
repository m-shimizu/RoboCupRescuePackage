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
#include_directories(${GAZEBO_INCLUDE_DIRS} ${CMAKE_CURRENT_BINARY_DIR}/msgs)
#link_directories(${GAZEBO_LIBRARY_DIRS} ${CMAKE_CURRENT_BINARY_DIR}/msgs)

################################################
# Build flipper control message
include_directories(${CMAKE_CURRENT_BINARY_DIR}/msgs)
link_directories(${CMAKE_CURRENT_BINARY_DIR}/msgs)
add_subdirectory(msgs)

################################################
# Build the USARGazebo plugin
add_library(USARGazebo SHARED USARGazebo.cc)
target_link_libraries(USARGazebo flipper_control_msgs ${GAZEBO_LIBRARIES})
add_dependencies(USARGazebo flipper_control_msgs encoder_msgs)
#If need boot and sdf libraries, add ${Boost_LIBRARIES} ${SDF_LIBRARIES} in link

# Build the Diff-Drive plugin for RoboCup2016
add_library(DiffDrivePlugin_rc2016 SHARED DiffDrivePlugin_rc2016.cc)
target_link_libraries(DiffDrivePlugin_rc2016 ${GAZEBO_LIBRARIES})

# Build the Skid-Steer plugin for RoboCup2016
add_library(SkidSteerDrivePlugin_rc2016 SHARED SkidSteerDrivePlugin_rc2016.cc)
target_link_libraries(SkidSteerDrivePlugin_rc2016 ${GAZEBO_LIBRARIES})
add_dependencies(SkidSteerDrivePlugin_rc2016 encoder_msgs)

# Build the Crawler-Flipper Arm plugin
add_library(crawler_robot_1DEG SHARED crawler_robot_1DEG.cc)
target_link_libraries(crawler_robot_1DEG flipper_control_msgs ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(crawler_robot_1DEG flipper_control_msgs)

################################################
# Build the example a set of USARGazebo-WorldPlugin(Subscriber) and usarsim_pub program
#add_library(USARGazeboW SHARED USARGazebo-WorldPluginSample.cc)
#target_link_libraries(USARGazeboW ${GAZEBO_LIBRARIES})
#add_executable(usarsim_pub usarsim_pub.cc)
#target_link_libraries(usarsim_pub ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})

###############################################
# Build the optional tool
add_executable (robot_teleop robot_teleop.cc)
target_link_libraries(robot_teleop flipper_control_msgs ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(robot_teleop flipper_control_msgs)
