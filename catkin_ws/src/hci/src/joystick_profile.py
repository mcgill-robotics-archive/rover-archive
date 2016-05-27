import rospy

## List of legal parameter names for button functions  
param_names = [# mode selection
               "/joystick/drive_mode",
               "/joystick/arm_base_mode",
               "/joystick/camera_mode",

               # drive mode:
               "/joystick/toggle_point_steer",
               "/joystick/ackreman_moving",
               "/joystick/point_steer",
               "/joystick/ackreman",

               "joystick/prev_cam",
               "joystick/next_cam",

               # arm mode:
               "/joystick/next_arm_joint",
               "/joystick/prev_arm_joint",
               "/joystick/coord_system",

               "/logitech/base",
               "/logitech/diff1",
               "/logitech/diff2",
               "/logitech/end"]

## Provides a simple mechanism to create button mapping on the controller.
#
# Reads ROS parameters to map a button number to a named functionnality. 
# This class reads the controller state and exposes the buttons under a 
# different name.
#
# Parameter names allowed are:
#
#<code>
#
#    # mode selection
#    "/joystick/drive_mode",
#    "/joystick/arm_base_mode",
#    "/joystick/camera_mode",
#
#    # in drive mode:
#    "/joystick/toggle_point_steer",
#    "/joystick/ackreman_moving",
#    "/joystick/point_steer",
#    "/joystick/ackreman",
#
#    # in camera mode:
#    # changes which camera is displayed in the main port 
#    "/joystick/prev_cam", 
#    "/joystick/next_cam",
#
#    # in arm mode:
#    "/joystick/next_arm_joint",
#    "/joystick/prev_arm_joint",
#    "/joystick/coord_system",
#
#    buttons only on the logitech controller
#    "/logitech/base",
#    "/logitech/diff1",
#    "/logitech/diff2",
#    "/logitech/end"
#
#</code>
#
class JoystickProfile:

    ## Constructor reads the parameters from the ROS master and establishes the
    # button name relationship.
    #
    # @param controller The JoystickController object to take the buttons from.
    #
    def __init__(self, controller):

        ## Handle to the controller object
        self.controller = controller

        ## The named button states
        self.param_value = {}

        ## Map representation of the relationship between name and button number
        self.mapping = {}
        for param in param_names:
            self.mapping[param] = rospy.get_param(param, None)

    ## Update the values in the map.
    #
    # Should be called every time JoystickController::update() is called to 
    # refresh the map.
    def update_values(self):
        for param in param_names:
            value = self.mapping[param]
            if value == "1":
                self.param_value[param] = self.controller.b1
            elif value == "2":
                self.param_value[param] = self.controller.b2 
            elif value == "3":
                self.param_value[param] = self.controller.b3 
            elif value == "4":
                self.param_value[param] = self.controller.b4 
            elif value == "5":
                self.param_value[param] = self.controller.b5 
            elif value == "6":
                self.param_value[param] = self.controller.b6 
            elif value == "7":
                self.param_value[param] = self.controller.b7 
            elif value == "8":
                self.param_value[param] = self.controller.b8 
            elif value == "9":
                self.param_value[param] = self.controller.b9 
            elif value == "10":
                self.param_value[param] = self.controller.b10 
            elif value == "11":
                self.param_value[param] = self.controller.b11 
            elif value == "12":
                self.param_value[param] = self.controller.b12
            elif value == "hat_top":
                self.param_value[param] = self.controller.hat_top
            elif value == "hat_left":
                self.param_value[param] = self.controller.hat_left
            elif value == "hat_right":
                self.param_value[param] = self.controller.hat_right
            elif value == "hat_down":
                self.param_value[param] = self.controller.hat_down
            else:
                self.param_value[param] = None
