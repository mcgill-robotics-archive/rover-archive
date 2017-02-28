from PyQt5.QtCore import QObject

from controller.arm_controller import ArmController
from controller.camera_controller import CameraController
from controller.drive_controller import DriveController
from controller.joystick.joystick_controller import JoystickController
from controller.navigation_controller import NavigationController


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
        self.arm_controller = ArmController(main_view.arm_view, self)
        self.joystick_master.addController("Arm", self.arm_controller)
        ## Navigation controller, subscribes to the ahrs publisher
        self.navigation_controller = NavigationController(self)

        ## Camera controller
        self.camera_controller = CameraController(self)
        self.camera_controller.add_screen(main_view.nav_screen.left_wheel)
        self.camera_controller.add_screen(main_view.nav_screen.right_wheel)
        self.camera_controller.add_screen(main_view.nav_screen.bottom_cam)

        self.arm_controller.updateMotorEnable.connect(main_view.drive_view.updateMotorEnable)

        self.drive_controller.wheelStatusUpdate.connect(main_view.drive_view.updateMotorStatus)
        self.drive_controller.forceControlsUpdate.connect(main_view.drive_view.displayDriveSettings)
        main_view.drive_view.controlsUpdated.connect(self.drive_controller.setDriveSetting)

        self.navigation_controller.updatePitch.connect(main_view.navigation_view.handle_new_pitch)
        self.navigation_controller.updateRoll.connect(main_view.navigation_view.handle_new_roll)
        self.navigation_controller.updateYaw.connect(main_view.navigation_view.handle_new_yaw)
        self.navigation_controller.updateAttitude.connect(main_view.pose_display.update_pose)
        self.navigation_controller.updatePosition.connect(main_view.map.add_point)

    def __del__(self):
        self.joystick_master.stop()
