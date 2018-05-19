# arm\_ros\_control

This package contains the launch and configuration files for running
the controllers for the joints of the arm and simulating it in Gazebo.

## Launch
To launch Gazebo and the controllers :

```bash
roslaunch arm_simulate arm.launch
```

Unpause the simulation by pressing Space

## Send a command to the controllers
To send a command to a joint controller, run rqt_gui with the following
commands:

```bash
rosrun rqt_gui rqt_gui
```

Add a "Message Publisher" plugin (plugins > topics > message publisher)

To see which controllers are available, run the following command:

```bash
rostopic list | grep command
```

Choose one or multiple controllers and add them to rqt_gui.

eg.
```bash
/arm/base_yaw_position_controller/command
```

Finally enable the publisher and set the data field to the desired value.
