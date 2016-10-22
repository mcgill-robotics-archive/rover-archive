import rospy
from PyQt5.QtCore import QObject, pyqtSlot
from PyQt5.QtWidgets import QWidget

from controller.joystick.joystick_acquisition import JoystickAcquisition
from controller.joystick.joystick_base import JoystickBase
from controller.joystick.joystick_data import JoystickData


class JoystickController(QObject):
    def __init__(self, widget=None, parent=None):
        super(JoystickController, self).__init__(parent)

        self.available_controllers = {}
        self.active_controller = None
        try:
            self.acquisition = JoystickAcquisition(self)
            self.acquisition.start()
        finally:
            rospy.logerr("Starting joystick acquisition failed")
            pass

        self.mode_widget = widget
        self.mode_widget.changeMode.connect(self.setActiveJoystick)

    @pyqtSlot(str)
    def setActiveJoystick(self, name):
        print("Changing active joystick")
        if name is not None:
            try:
                new_controller = self.available_controllers[name]
            except KeyError as e:
                print(e)
                raise e

            if self.active_controller is not None:
                self. self.acquisition.joystickDataUpdated.disconnect(self.active_controller.handle_joystick_data)

            self.active_controller = new_controller
            self.acquisition.joystickDataUpdated.connect(self.active_controller.handle_joystick_data)

    @pyqtSlot(str, JoystickBase)
    def addController(self, name, controller):
        assert isinstance(controller, JoystickBase)
        if name is not "" and controller is not None:
            try:
                existing_controller = self.available_controllers[name]
                raise ValueError("Name already exists, addition impossible")
            except KeyError as e:
                self.available_controllers[name] = controller
                self.mode_widget.addMode(name)
