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


API Documentation
---
Jenkins generated doxygen documentation: 
http://dev.mcgillrobotics.com:8080/job/rover_all/doxygen/

Bagging using topics file
---

You can bag a specific set of topics (our common configuration is in the topics
file). You use the following (based on compsysy setup)
```bash
bag record <shortcuts>
```
