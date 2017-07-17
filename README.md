McGill Robotics Rover Project
---

This repository contains the ROS workspace used for the Mars Rover Project.



Build Status
------------

[master]: http://dev.mcgillrobotics.com/buildStatus/icon?job=rover_master
[master url]: http://dev.mcgillrobotics.com/job/rover_master

[dev]: http://dev.mcgillrobotics.com/buildStatus/icon?job=rover_dev
[dev url]: http://dev.mcgillrobotics.com/job/rover_dev


| Branch   | Status                  |
|:--------:|:-----------------------:|
| `master` | [![master]][master url] |
| `dev`    | [![dev]][dev url]       |


System Requirements
---
- Ubuntu 16.04 Xenial Xerus
- ROS Kinetic Kame
- McGill Robotics [Compsys](https://github.com/mcgill-robotics/compsys) Setup

Building and Linting
---
We use catkin_tools to build our system. **Both the build and lint step must
pass before changes can be merged.** 

Get to the `catkin_ws` directory by issuing
```bash
roscd
```
**Build Step**
```bash
catkin build
```
**Lint Step**
```bash
catkin lint --explain -W2 src --ignore target_name_collision --skip-pkg rosserial_tivac --strict
```

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
### Step 1 - Launching the Rover
This has become fairly streamlined. In order to launch the rover, simply
ssh into the rover and then launch tmuxinator:

```bash
ssh robotics@rover
```
```bash
tmuxinator start rover
```

In order to control the rover from your own computer, you need to tell your
computer where ROS\_MASTER is then you simply launch HCI.


On your own machine:
```bash
export ROS_MASTER_URI=http://rover:11311
```


At this point, as a sanity check, you should be able to see the rostopics
from the ground station, i.e. `rostopic list` should show the topics from
control.


Finally, we need to launch the control station:
```bash
roslaunch hci hci.launch
```

Bagging using topics file
---

You can bag a specific set of topics (our common configuration is in the topics
file). You use the following (based on compsysy setup)
```bash
bag record <shortcuts>
```
