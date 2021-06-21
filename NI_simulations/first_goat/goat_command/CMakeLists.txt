cmake_minimum_required(VERSION 3.0.2)
project(goat_command)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  roscpp
  rospy
  std_msgs
)

catkin_package(
  LIBRARIES goat_command
  CATKIN_DEPENDS gazebo_ros roscpp rospy std_msgs
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(give_command_node src/give_command.cpp)

target_link_libraries(give_command_node
   ${catkin_LIBRARIES}
 )
