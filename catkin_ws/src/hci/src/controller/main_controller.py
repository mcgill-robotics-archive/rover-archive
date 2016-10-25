from PyQt5.QtCore import QObject

from controller.drive_controller import DriveController
from controller.joystick.joystick_controller import JoystickController


class MainController(QObject):
    """!@brief Main controller for the application

    Responsible for linking the different lower level controllers together
    and to their views. This should be the main router for the signals and
    slot mechanisms.
    """

    def __init__(self, main_view, parent=None):
        """!@brief Constructor.

        Initialize the module controllers and link signals

        @param self Python object pointer
        @param main_view MainView object previously instantiated to link signals
        @param parent QObject Qt hierarchy parent
        """

        super(MainController, self).__init__(parent)

        ## Joystick master controller, handles all joystick operations
        self.joystick_master = JoystickController(main_view.joystick_mode_widget, self)
        ## Drive controller, links to ROS  drive systems
        self.drive_controller = DriveController(self)
        self.joystick_master.addController("Drive", self.drive_controller)

        self.drive_controller.wheelStatusUpdate.connect(main_view.drive_view.updateMotorStatus)
        self.drive_controller.forceControlsUpdate.connect(main_view.drive_view.displayDriveSettings)
        main_view.drive_view.controlsUpdated.connect(self.drive_controller.setDriveSetting)
