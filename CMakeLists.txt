cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

#### CMakeLists for collision_map_creator
# SEE CMakeLists-Collision.txt

#### CMakeLists for other plugin tutorial

find_package(Boost REQUIRED COMPONENTS system)
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
  pkg_check_modules(OGRE OGRE)
  pkg_check_modules(OGRE-Terrain OGRE-Terrain)
endif()
include_directories(${GAZEBO_INCLUDE_DIRS} ${OGRE_INCLUDE_DIRS} ${OGRE-Terrain_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS} ${OGRE_LIBRARY_DIRS})

add_library(USARGaz SHARED USARGaz1.cc)
target_link_libraries(USARGaz ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} ${PROTUBUF_LIBRARIES})

#add_library(hello_world SHARED hello_world.cc)
#target_link_libraries(hello_world ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})
#add_library(model_push SHARED model_push.cc)
#target_link_libraries(model_push ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})
#add_library(factory SHARED factory.cc)
#target_link_libraries(factory ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} ${PROTUBUF_LIBRARIES})
#add_library(world_edit SHARED world_edit.cc)
#target_link_libraries(world_edit ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})
#add_library(system_gui SHARED system_gui.cc)
#target_link_libraries(system_gui ${GAZEBO_LIBRARIES} ${OGRE_LIBRARIES})
#add_library(world_edit_velcmd SHARED world_edit_velcmd.cc)
#target_link_libraries(world_edit_velcmd ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})
#add_executable (robot_teleop robot_teleop.cc)
#target_link_libraries(robot_teleop ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} ${SDF_LIBRARIES})
