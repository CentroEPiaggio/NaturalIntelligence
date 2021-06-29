## Second Goat Robot Model

This folder includes 3 ROS packages to create and use the second version of a goat robot model built with 8 *qbmove Advance*

### How to spawn the robot

- Copy the whole simulation folder *second_goat* in your catkin_ws/src 
- ``` $ catkin_make ```

To visualize the robot in Rviz
- ``` $ roslaunch second_goat_description rviz.launch ```

To spawn the robot in Gazebo
- ``` $ roslaunch second_goat_gazebo second_goat_spawn.launch ```
``
