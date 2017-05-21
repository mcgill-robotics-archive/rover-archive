McGill Robotics Rover Project
---

This repository contains the ROS workspace used for the Mars Rover Project.



Build Status
------------

[master]: http://dev.mcgillrobotics.com:8080/buildStatus/icon?job=rover_master
[master url]: http://dev.mcgillrobotics.com:8080/job/rover_master

[dev]: http://dev.mcgillrobotics.com:8080/buildStatus/icon?job=rover_dev
[dev url]: http://dev.mcgillrobotics.com:8080/job/rover_dev


| Branch   | Status                  |
|:--------:|:-----------------------:|
| `master` | [![master]][master url] |
| `dev`    | [![dev]][dev url]       |


Initial Setup
---
Simply follow the compsys setup and select Rover when prompted.

Running the Rover
---
### First time connection
**Note that if it is your first time connecting, you need to do a few extra
steps**:

Add the rover to your computer's hosts: add 
```bash
192.168.3.252 rover
```
to your `/etc/hosts` file. You will now be able to ssh into the rover with
```bash
ssh robotics@rover
```
Now, ssh into the rover, and on the rover, add your ip and user pair to
the rover's host file. You can see your ip by issuing `ip addr show` (on your
computer). You can see your username by issuing `whoami`, again, on your own
computer. Now on the rover, add the following line to `/etc/hosts`:
```bash
<your-computer-ip> <your-computer-name>
```

## Recurring Steps
On your computer in the root of the rover repository:
```bash
./scripts/control
```

Next, in a separate terminal or tmux window, ssh into the rover:
```bash
ssh robotics@rover
```

Now you need to tell the rover where ROS_MASTER is:
```bash
export ROS_MASTER_URI=http://<your-computer's-name>:11311
```

At this point, as a sanity check, you should be able to see the rostopics
from the base station, i.e. `rostopic list` should show the topics from control.

Finally, we need to run the Arduinos on the rover:
```bash
roslaunch arduino drive.launch
```


API Documentation
---
Jenkins generated doxygen documentation: 
http://dev.mcgillrobotics.com:8080/job/rover_all/doxygen/

Note on Building
---
We use `catkin_make` to build our main workspace (`catkin_ws`). However we use
`catkin build` to build our MoveIt workspace (`moveit_ws`).
The `init.sh` script builds both the workspaces for you (if you decide to
setup your MoveIt! workspace).
