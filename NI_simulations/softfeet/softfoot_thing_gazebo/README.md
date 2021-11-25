# softfoot_thing_gazebo

This package contains the plugin that attempts to correctly simulate the foot in Gazebo. It is basically a ModelPlugin that closes the kinematic chains once the URDF is converted into SDF by Gazebo.

## Getting Started

### Prerequisites

The parent repository of this package should be cloned and built in your catkin workspace.

### Running a simulation

A gazebo simulation launch is provided in the package `softfoot_thing_description` as this package is exclusively for the plugin.

To run a foot only simulation in Gazebo run the following command:

`roslaunch softfoot_thing_description pisa_softfoot.launch`

A floating (free to fall) single foot simulation can be launched by

`roslaunch softfoot_thing_description floating_softfoot.launch`

Finally, four floating (free to fall) feet simulation can be launched by

`roslaunch softfoot_thing_description four_softfoot.launch`
