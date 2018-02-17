# arm\_control

This package contains the MoveIt files for the arm, including the srdf, as well as other control scripts.

## Launch
To launch rviz with the arm model: (requires the gazebo simulation to be running at the same time)

```bash
roslaunch arm_control moveit.launch
```

To launch the topic remapper:

```bash
roslaunch arm_control tiva_topic_remap.launch
```

