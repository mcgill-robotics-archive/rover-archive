# arm\_description

This package contains the URDF file and meshes for the arm, as well as the configuration and launch files required to visualize it in rviz

## Launch
To launch rviz with the arm model and state publisher GUI:

```bash
roslaunch arm_description display.launch
```

Launch parameters:

- `gui:=False`: Launch without the state publisher gui.
