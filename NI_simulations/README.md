# Natural Intelligence Simulations

ROS packages for Natural Intelligence project H2020 simulations.

## Overview

The main simulation folders are:

##### anymal_c

Simple ANYmal C urdf version to spawn it in differents world created for the NI environments (dune, forest, grassland)

- *anymal_c_description* →
ROS package with the ANYmal C urdf and related meshes
	
- *anymal_c_gazebo* →
ROS package with the Natural Intelligence wolrds, related models and launch file

##### first_goat

Simple goat robot urdf model to simulate NI Integrator 3

- *goat_command* →
ROS package useful to command each goat joint through the [`plugin manager`](https://github.com/Richi90/ROS-Gazebo-plugin-qbmove)

- *goat_description* →
ROS package with the first goat urdf version with the floating base, 8 links and 8 joints
	
- *goat_gazebo* →
ROS package with folders and files useful to spawn the goat_robot in a Gazebo world

- *goat_plugin* 
	- *sea_plugin* → ROS package to integrate a Serial Elastic Actuator for each joint
	- *plugin_manager* → ROS package to easly manage joint control

## Getting Started

### Installing

To install the packages in this repo just clone it into your catkin_ws, make sure to clone also the external dependencies and then `catkin_make`.
