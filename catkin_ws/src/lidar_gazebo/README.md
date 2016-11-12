# lidar\_gazebo

This package contains the launch and world configuration files for running
the lidar in the Gazebo simulation environment.

## Launch
To launch the Gazebo simulator with the default world:

```bash
roslaunch lidar_gazebo lidar.launch
```

## View Scan results in rviz
To view the output from the Gazebo simulation in rviz, issue the following
command:

```bash
rosrun rviz rviz -d `rospack find lidar_description`/lidar.rviz
```
