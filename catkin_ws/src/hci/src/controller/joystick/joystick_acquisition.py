"""!@brief Joystick acquisition using pygame library.
"""
import pygame
import time
import rospy
from PyQt5.QtCore import QThread
from PyQt5.QtCore import pyqtSignal

from model.joystick_data import JoystickData


class JoystickAcquisition(QThread):
    """!@brief Acquire joystick data and signal with available data
    """
    joystickDataUpdated = pyqtSignal(object)

    def __init__(self, parent=None):
        """!@brief Constructor, will start acquisition if joystick found.

        Find joystick to use, should only be one joystick connected.
        Will span new thread and run acquisition in loop. Signal is emitted
        with joystick data at every loop iteration. Loop runs at 100 Hz
        frequency.

        In calling class:
        @code{.py}
        acquisition = JoystickAcquisition(self)
        acquisition.start()
        @endcode

        This call will be forwarded to the run() function and start the main
        acquisition loop and emit the signal with every iteration.

        @param self Python object pointer
        @param parent QWidget parent for object hierarchy
        """

        super(JoystickAcquisition, self).__init__(parent)
        self.data = JoystickData()

        pygame.init()
        pygame.joystick.init()

        if pygame.joystick.get_count() == 1:
            # confirm there is only one joystick, possible todo: allow device
            # identification to be passed in constructor to have multiple
            # controllers
            self.controller = pygame.joystick.Joystick(0)
            self.controller.init()
        else:
            self.controller = None

        if self.controller is None:
            raise AssertionError("Joystick not initialized properly, make sure you have one connected")

    def update(self):
        """!@brief Single cycle acquisition method

        Update internal structure with values from joystick buttons and
        axis values

        @param self Python object pointer
        """

        for an_event in pygame.event.get():
            try:
                if an_event.type == pygame.JOYBUTTONDOWN or an_event.type == pygame.JOYBUTTONUP:
                    self.data.b1 = self.controller.get_button(0)
                    self.data.b2 = self.controller.get_button(1)
                    self.data.b3 = self.controller.get_button(2)
                    self.data.b4 = self.controller.get_button(3)
                    self.data.b5 = self.controller.get_button(4)
                    self.data.b6 = self.controller.get_button(5)
                    self.data.b7 = self.controller.get_button(6)
                    self.data.b8 = self.controller.get_button(7)
                    self.data.b9 = self.controller.get_button(8)
                    self.data.b10 = self.controller.get_button(9)
                    self.data.b11 = self.controller.get_button(10)
                    self.data.b12 = self.controller.get_button(11)
                elif an_event.type == pygame.JOYAXISMOTION:
                    self.data.a1 = self.controller.get_axis(0)
                    self.data.a2 = -1 * self.controller.get_axis(1)
                    self.data.a3 = self.controller.get_axis(2)
                    self.data.a4 = self.controller.get_axis(3)
                else:
                    self.data.hat = self.controller.get_hat(0)
                    if self.data.hat == (-1, 0):
                        self.data.hat_left = True
                    elif self.data.hat == (0, 1):
                        self.data.hat_top = True
                    elif self.data.hat == (0, -1):
                        self.data.hat_down = True
                    elif self.data.hat == (1, 0):
                        self.data.hat_right = True
            except pygame.error:
                pass
            finally:
                pass

    def run(self):
        """!@brief Function will be run when thread started.

        Runs loop of acquisition and signal emission.

        @param self Python object pointer
        """

        while not rospy.is_shutdown():
            # todo add hot swap capability maybe
            self.update()
            self.joystickDataUpdated.emit(self.data)
            time.sleep(0.01)
