# softfoot_thing_description

This package contains all the basics for the foot model in ROS.

## Getting Started

### Prerequisites

The parent repository of this package should be cloned and built in your catkin workspace.

### Foot model
The basic robot model of the SoftFoot to be attatched to the leg of any legged robot (e.g. Anymal) is `softfoot_thing.urdf.xacro` which can be found in the folder `model`.

To mount a SoftFoot on your robot, add the following to the `urdf.xacro` of your robot:

```xml
<!-- FOOT 1 -->
<xacro:softfoot_thing name="<softfoot-name>_<softfoot-id>" parent="<parent-link>">
    <origin xyz="0 0 0" rpy="0 0 0"/>
</xacro:softfoot_thing>
```

Here `<softfoot-name>_<softfoot-id>` and `<parent-link>` are the name of the foot and the link to which it is attatched respectively. It is advised to follow some general guidelines in choosing `<softfoot-name>` and `<softfoot-id>`: for this please refer to the section *Visualizing SoftFeet on RViz* of the package `softfoot_thing_visualization`.

### Running a simulation

Several gazebo simulation launch files are provided within this package. The `softfoot_thing_gazebo` package is exclusively for the plugin.

A fixed single foot simulation in Gazebo can be launched as follows:

`roslaunch softfoot_thing_description pisa_softfoot.launch`

A floating (free to fall) single foot simulation can be launched by

`roslaunch softfoot_thing_description floating_softfoot.launch`

Finally, four floating (free to fall) feet simulation can be launched by

`roslaunch softfoot_thing_description four_softfoot.launch`
