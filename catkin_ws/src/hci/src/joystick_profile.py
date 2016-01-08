import rospy
               # mode selection
param_names = ["/joystick/drive_mode",
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
               "/joystick/arm_mode",

               # logitech specific dofs
               "/logitech/shoulder",
               "/logitech/elbow",
               "/logitech/wrist",
               "/logitech/roll",
               "/logitech/grip",
               "/logitech/base"]


class JoystickProfile:
    def __init__(self, controller):
        self.controller = controller
        self.param_value = {}
        self.mapping = {}
        for param in param_names:
            self.mapping[param] = rospy.get_param(param, None)

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