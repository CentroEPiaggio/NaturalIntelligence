cmake_minimum_required(VERSION 2.8.3)
project(softfoot_thing_gazebo)

add_definitions(-std=c++11)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  std_msgs
  message_generation
  gazebo_ros
)

find_package(yaml-cpp REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS std_msgs message_runtime gazebo_ros
  LIBRARIES ${PROJECT_NAME}_plugin
)

set(OBJECTS_SOURCE_FILES
  src/utils/parsing_utilities.cpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Softfoot Gazebo Plugin
find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

add_library(${PROJECT_NAME}_plugin  src/softfoot_gazebo_plugin.cpp
  include/${PROJECT_NAME}/softfoot_gazebo_plugin.hpp
)
# add_dependencies(${PROJECT_NAME}_plugin
# )
target_link_libraries(${PROJECT_NAME}_plugin
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)

# Installation
install(TARGETS ${PROJECT_NAME}_plugin
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})