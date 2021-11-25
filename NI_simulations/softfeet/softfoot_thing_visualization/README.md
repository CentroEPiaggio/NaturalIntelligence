# softfoot_thing_visualization

This package contains all the necessary utilities to work on ROS with a real SoftFoot. It provides basically two ros nodes for
* Calibration - Reads IMU measurements from a foot in stable position and saves a `.yaml` file with the necessary details.
* Visualization - Calibrates the foot on the fly or loads pre-existing calibration data and estimates current joint states from IMU measurements.

### Prerequisites

The parent repository of this package should be cloned and built in your catkin workspace.

### Calibrating a SoftFoot

The feet are provided with default calibration files which can be enough for good estimation. Nevertheless, re-calibration might be required.

To this end, connect the foot to your computer (one foot at a time), keep the foot on a flat surface and hold the leg (tube) perpendicular to the flat surface. Then launch the following, in order:
```
roslaunch softfoot_thing_visualization pisa_softfoot_calibration_base.launch foot_id:=<id on foot (1, 2, 3 or 4)>
roslaunch softfoot_thing_visualization pisa_softfoot_calibration.launch
```

You will be asked to type the **Name** and **ID** of the foot. A good rule would be to give as **Name** a basic identifier (e.g. softfoot or footthing), which will be the same for all feet of the robot, and the **ID** should be the `board_id` of the real foot.

A file named `Name_ID.yaml` file will be saved to the `config` folder of the current package. Then, make sure to add it to your foot visualization launch file so to load its parameters into the ROS parameter server.

### Visualizing SoftFeet on RViz

#### Prerequisites

##### Setup the foot on your robot

Setting up the robot model properly is vital for the estimated joint states to be published into the correct topic so that they can be sourced by the `joint_state_publisher`.

> For example:
> Suppose you calibrated a SoftFoot with **Name** = softfoot and **ID** = 4.
> When adding the model of that particular foot to the urdf.xacro of your robot, make sure to give that xacro the name softfoot_4.
> ```xml
> <xacro:softfoot_thing name="softfoot_4" parent="<parent-link>">
>     <origin xyz="0 0 0" rpy="0 0 0"/>
> </xacro:softfoot_thing>
> ```

##### Specify the visualization parameters

The file `pisa_softfoot_viz.yaml` in the folder `configs` of this package gives the user some control over the visualization of the feet. Most of the parameters therewithin are self explanatory, but the following table explains them for the sake of completeness.

| Parameter             | Type          | Description  |
| ----------------------|:-------------:| -------------|
| `calibrate_online`    | `bool`        | If `true` an online calibration will be performed for all the feet name specified in `connected_feet_name` with ids specified in `connected_feet_ids`. N.B. : If calibrating online, the feet should be on a flat surface and leg vertical.|
| `use_filter`          | `bool`        | If `true` the acceleration and gyroscope measurements will be prelimilarly smoothed using the low pass filter defined by the parameter `low_pass_filter`. |
| `use_gyro`            | `bool`        | If `true` the gyro measurements will be integrated and fused to the joint angle estimations from accelerations through a complementary filter. |
| `publish_leg_pose`    | `bool`        | If `true` a raw estimate of the leg pose will be published to the joint states. This might not be useful in case the leg angle of your robot is already provided by other means (robot joint_states). |
| `connected_feet_name` | `string`      | A sting containing the basic identifier of the feet in your robot model (**Name**). |
| `connected_feet_ids`  | `int[]`       | IMPORTANT: An array of integers containing the IDs of the (connected) feet in your robot model (**ID**). |

Some other parameters can also be found in the configuration file, but these are relative to future work and thus can be safely ignored.

##### Setup a visualization launch file

Get familiar with the example launch file `pisa_softfoot_visualization.launch`. Then, in a launch file load into the ROS parameter server the following configuration files:
* `softfoot_thing_visualization/configs/pisa_softfoot_viz.yaml` - file with visualization options.
* The calibration yaml files (e.g. `softfoot_thing_visualization/configs/softfoot_1.yaml`) for each foot.

Source the feet joint states into the base `joint_states` topic. For example, if you specified earlier **Name** = softfoot:
```xml
<node name="joint_state_publisher" pkg="joint_state_publisher" type="joint_state_publisher">
    <rosparam param="source_list">[/softfoot_1/joint_states, /softfoot_2/joint_states, /softfoot_3/joint_states, /softfoot_4/joint_states]</rosparam>
</node>
```

Add the launch files for communication with the IMUs. For example, if all four feet are connected:
```xml
<include file="$(find nmmi_examples)/launch/SoftFoot_5_IMU_boards_chain.launch"></include>
```

Instead, if only one foot is connected, add the provided launch file accordingly. For example, in case of foot 1 connected:
```xml
<include file="$(find nmmi_examples)/launch/SoftFoot_IMU_1_chain.launch"></include>
```

N.B. : If you have only 2 or 3 feet connected, you might need assistance to setup a communication launch file in `NMMI/ROS-NMMI`.

Finally, add the foot joint states estimator to the launch:
```xml
<!-- Joints Estimator -->
<group unless="$(arg gui)">
    <node name="js_estimator" pkg="softfoot_thing_visualization" type="softfoot_thing_visualization_joints_estimator" required="true" output="screen"/>
</group>
```

#### Visualizing

Now, if you connect to a USB hub the feet specified in the parameter `connected_feet_ids` in the file `softfoot_thing_visualization/configs/pisa_softfoot_viz.yaml` and run the created roslaunch file, you should be able to see the joint states of the feet in the topic `joint_states`.