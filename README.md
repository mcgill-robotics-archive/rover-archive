McGill Robotics Rover Project
---

This repository contains the ROS workspace used for the Mars Rover Project.

Build Status
------------

[master]: http://dev.mcgillrobotics.com:8080/buildStatus/icon?job=rover_master
[master url]: http://dev.mcgillrobotics.com:8080/job/rover_master

[dev]: http://dev.mcgillrobotics.com:8080/buildStatus/icon?job=rover_dev_kinetic
[dev url]: http://dev.mcgillrobotics.com:8080/job/rover_dev_kinetic

[all]: http://dev.mcgillrobotics.com:8080/buildStatus/icon?job=rover_kinetic
[all url]: http://dev.mcgillrobotics.com:8080/job/rover_kinetic

| Branch   | Status                  |
|:--------:|:-----------------------:|
| `master` | [![master]][master url] |
| `dev`    | [![dev]][dev url]       |
| `all`    | [![all]][all url]       |

Initial Setup
---
Issue the following commands to get you up and running with the most up to date
rover setup:
```bash
./init.sh
```

API Documentation
---
Jenkins generated doxygen documentation: 
http://dev.mcgillrobotics.com:8080/job/rover\_all/doxygen/

Note on Building
---
We use `catkin_make` to build our main workspace (`catkin_ws`). However we use
`catkin build` to build our MoveIt workspace (`moveit_ws`).
The `init.sh` script builds both the workspaces for you (if you decide to
setup your MoveIt! workspace).
