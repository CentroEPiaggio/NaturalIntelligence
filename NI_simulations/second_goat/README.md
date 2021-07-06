## Second Goat Robot Model

This folder includes 2 ROS packages to create and use the second version of a goat robot model built with 8 *qbmove Advance*

### Prerequisites

The above packages are tested on ROS Noetic and the main external dependency is the following:

* [`goat_plugin`](https://github.com/CentroEPiaggio/NaturalIntelligence/tree/main/NI_simulations/goat_plugin)

Used to integrate the serial elasti actuator in each joint and command the robot via the plugin manager.

### How to spawn the robot

- Copy the whole simulation folder *second_goat* in your catkin_ws/src 
- ``` $ catkin_make ```

To visualize the robot in Rviz
- ``` $ roslaunch second_goat_description rviz.launch ```

To spawn the robot in Gazebo
- ``` $ roslaunch second_goat_gazebo second_goat_spawn.launch ```

**Warning**: if you want launch a custumized world that uses gazebo world models, before the command above run:

-``` $ export GAZEBO_MODEL_PATH=~/catkin_ws/src/second_goat/second_goat_gazebo/models:$GAZEBO_MODEL_PATH ```
