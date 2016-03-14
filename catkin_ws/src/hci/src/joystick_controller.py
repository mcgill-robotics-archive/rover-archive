#
#  Main file for Joystick Controller
#
#  @author David Lavoie-Boutin

try:
    import os
    import sys
    import thread
    import time
    from pygame import locals
    import pygame
except ImportError:
    print "Missing libraries for joystick controller, exiting"
    sys.exit()

## Class abstracting the joystick controller
#
class JoystickController(object):
    
    ## Constructor initialises connection to the joystick 
    # 
    # Only one joystick must be present when initialising object.
    # If more than one joystick exist, the constructor will create a Null controller.
    def __init__(self):
        pygame.init()
        pygame.joystick.init()

        self.b1 = False
        self.b2 = False
        self.b3 = False
        self.b4 = False
        self.b5 = False
        self.b6 = False
        self.b7 = False
        self.b8 = False
        self.b9 = False
        self.b10 = False
        self.b11 = False
        self.b12 = False

        self.hat = 0

        self.hat_left = False
        self.hat_top = False
        self.hat_down = False
        self.hat_right = False

        self.a1 = 0
        self.a2 = 0
        self.a3 = 0
        self.a4 = 0

        if pygame.joystick.get_count() == 1:
            # confirm there is only one joystick, possible todo: allow device 
            # identification to be passed in constructor to have multiple 
            # controllers
            self.controller = pygame.joystick.Joystick(0)
            self.controller.init()
        else:
            self.controller = None

    ## Reset the state of all internal button registers to zero
    #
    def clear_buttons(self):
        self.b1 = False
        self.b2 = False
        self.b3 = False
        self.b4 = False
        self.b5 = False
        self.b6 = False
        self.b7 = False
        self.b8 = False
        self.b9 = False
        self.b10 = False
        self.b11 = False
        self.b12 = False

        self.hat = 0

        self.hat_left = False
        self.hat_top = False
        self.hat_down = False
        self.hat_right = False

    ## Read the status of the controller and update internal registers.
    #
    # This method fetches the information form the controller and stores it in 
    # class members. If a changed occurred, the value will be updated; else the 
    # value will be the last one fetched.  
    def update(self):
        for anEvent in pygame.event.get():
            try:
                if anEvent.type == pygame.locals.JOYBUTTONDOWN:
                    self.b1 = self.controller.get_button(0)
                    self.b2 = self.controller.get_button(1)
                    self.b3 = self.controller.get_button(2)
                    self.b4 = self.controller.get_button(3)
                    self.b5 = self.controller.get_button(4)
                    self.b6 = self.controller.get_button(5)
                    self.b7 = self.controller.get_button(6)
                    self.b8 = self.controller.get_button(7)
                    self.b9 = self.controller.get_button(8)
                    self.b10 = self.controller.get_button(9)
                    self.b11 = self.controller.get_button(10)
                    self.b12 = self.controller.get_button(11)    
                elif anEvent.type == pygame.locals.JOYAXISMOTION:
                    self.a1 = self.controller.get_axis(0)
                    self.a2 = self.controller.get_axis(1)
                    self.a3 = self.controller.get_axis(2)
                    self.a4 = self.controller.get_axis(3)
                else:
                    self.hat = self.controller.get_hat(0)
                    if self.hat == (-1, 0):
                        self.hat_left = True
                    elif self.hat == (0, 1):
                        self.hat_top = True
                    elif self.hat == (0, -1):
                        self.hat_down = True
                    elif self.hat == (1, 0):
                        self.hat_right = True
            except pygame.error:
                pass
            finally:
                pass

if __name__ == "__main__":
    controller = JoystickController()
    while True:
        controller.update()
        print (controller.a1, controller.a2, controller.a3)
        