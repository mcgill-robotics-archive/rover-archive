"""!@brief Joystick data dispatcher"""

import rospy
from PyQt5.QtCore import QObject, pyqtSlot

from controller.joystick.joystick_acquisition import JoystickAcquisition
from controller.joystick.joystick_base import JoystickBase


class JoystickController(QObject):
    """!@brief Main controller class for joystick applications

    Owner of the joystick acquisition class and controls the dispatch of the
    joystick data. Class wishing to receive joystick data must register
    themselves to this class in order to eventually be activated.
    """

    def __init__(self, widget=None, parent=None):
        """!@brief Constructor. Initializes acquisition and connects the view
        components to the appropriate slots

        @param self Python object pointer
        @param widget The view widget, should have a member
        signal @code void changeMode(string) @endcode
        """

        super(JoystickController, self).__init__(parent)

        self._available_controllers = {}
        self._active_controller = None
        self.controller_Found = False
        try:
            self._acquisition = JoystickAcquisition(self)
            self._acquisition.start()
            self.controller_Found = True
        except AssertionError:
            rospy.logerr("Starting joystick acquisition failed")
            self.controller_Found = False

        self._mode_widget = widget
        self._mode_widget.changeMode.connect(self.setActiveJoystick)

    def stop(self):
        self._acquisition.terminate()

    @pyqtSlot(str)
    def setActiveJoystick(self, name):
        """!@brief Change which class receives joystick inputs.

        Disconnect previously active class and connect joystick acquisition to
        this class. The available classes are placed in a map, string keyed,
        so the name passed in argument must be a string present in the map of
        available controllers.

        @param self Python object pointer
        @param name String key of the controller to activate

        @throws KeyError is string not in map
        """
        if not self.controller_Found:
            rospy.logwarn("Joystick not connected. Ignoring command")
            return

        print("Changing active joystick")
        if name is not None:
            try:
                new_controller = self._available_controllers[name]
            except KeyError as e:
                print(e)
                raise e

            if self._active_controller is not None:
                self._acquisition.joystickDataUpdated.disconnect(self._active_controller.handle_joystick_data)

            self._active_controller = new_controller
            self._acquisition.joystickDataUpdated.connect(self._active_controller.handle_joystick_data)

    @pyqtSlot(str, JoystickBase)
    def addController(self, name, controller):
        """!@brief Registers a new controller for joystick capability

        Add a class to the list of control mode on the linked widget and add
        it to the map of available joystick classes.

        @param self Python object pointer
        @param name String displayed on the button to activate that class.
        Will also be the key in the map for that controller
        @param controller Controller class, must be an instance of
        JoystickBase class.
        """

        assert isinstance(controller, JoystickBase)
        if name is not "" and controller is not None:
            try:
                existing_controller = self._available_controllers[name]
                raise ValueError("Name already exists, addition impossible")
            except KeyError as e:
                self._available_controllers[name] = controller
                self._mode_widget.addMode(name)
