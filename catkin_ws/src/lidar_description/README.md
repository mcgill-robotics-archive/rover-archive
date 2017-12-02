# lidar\_description

This package contains all URDF descriptions and configuration files required to
visualize the lidar on its rotatable mount in rviz, as well as the model
required for simulation in Gazebo. See the `lidar_gazebo` package for Gazebo 
related launch and configuration files.

## Launching
To launch rviz with the lidar model and state publisher GUI:

```bash
roslaunch lidar_description lidar.launch
```

Launch parameters:

- `gui:=False`: Launch without the state publisher gui.
