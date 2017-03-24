# Arm Tiva Code

Build normally using `catkin_make` in the `catkin_ws` directory.
To flash the firmware, connect your board and issue one of the following
commands:
- For the CUI encoder code:
```bash
catkin_make arm_tiva_encoder_cui_flash
```
- For the magnetic encoder code:
```bash
catkin_make arm_tiva_encoder_magnetic_flash
```

Finally, to run the actual ros node, issue the following command:
```bash
rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0
```
