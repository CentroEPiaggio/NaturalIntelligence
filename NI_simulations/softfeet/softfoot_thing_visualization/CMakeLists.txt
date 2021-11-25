cmake_minimum_required(VERSION 2.8.3)
project(softfoot_thing_visualization)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  std_msgs
  message_generation
  geometry_msgs
  sensor_msgs
  tf
  tf_conversions
  eigen_conversions
  urdf
  kdl_parser
  qb_interface
  filters
  nmmi_msgs
)

find_package(yaml-cpp REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS std_msgs message_runtime
)

set(OBJECTS_SOURCE_FILES
  src/joints_estimator.cpp
  src/utils/parsing_utilities.cpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  /usr/include/eigen3
)

# NODE: Joint state estimator
add_executable(${PROJECT_NAME}_joints_estimator src/joints_estimator_node.cpp ${OBJECTS_SOURCE_FILES})
add_dependencies(${PROJECT_NAME}_joints_estimator ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_joints_estimator ${catkin_LIBRARIES} ${ROS_LIBRARIES} yaml-cpp)

# NODE: Foot calibrator
add_executable(${PROJECT_NAME}_calibration src/calibration_node.cpp ${OBJECTS_SOURCE_FILES})
add_dependencies(${PROJECT_NAME}_calibration ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_calibration ${catkin_LIBRARIES} ${ROS_LIBRARIES} yaml-cpp)