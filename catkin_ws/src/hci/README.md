# Human Computer Interaction (HCI) Package

Graphical user interface and tools for controlling and monitoring the McGill Robotics Rover. It interfaces with a usb Joystick to acquire speed input to control different joints. The package contains several general purpose classes: JoystickProfile, JoystickController, DrivePublisher, ArmPublisher.

The main executable for this package is hci.py which will launch the gui, acquire input from the joystick and publish commands for the control system.

## External ROS parameters

The main executable will ask for several parameters from the ROS master:

### Camera parameters

These parameters are all topic names that will be used to acquire compressed or raw video feeds for display or snapshot capture. The `*_dev` parameters is the device file pointer that will be passed in the service call to the video switcher.
```
camera/wide_angle       
camera/wide_angle_dev   
camera/wide_angle_hires 
camera/arm              
camera/arm_dev          
camera/arm_hires        
camera/pan_tilt         
camera/pan_tilt_dev     
camera/pan_tilt_hires   
```

### Joystick parameters

These parameters are used to create a mapping between button names and button functions. This has been done to allow the use of different joystick with different button layouts interchangeably.  

If joystick functions are not mapped to a button, they will simply not be available to the user. They will default to the 0 button (n/a)
```
# mode selection
/joystick/drive_mode
/joystick/arm_base_mode
/joystick/camera_mode

# in drive mode:
/joystick/toggle_point_steer
/joystick/ackreman_moving
/joystick/point_steer
/joystick/ackreman

# in camera mode:
# changes which camera is displayed in the main port 
/joystick/prev_cam
/joystick/next_cam

# in arm mode:
/joystick/next_arm_joint
/joystick/prev_arm_joint
/joystick/coord_system
```