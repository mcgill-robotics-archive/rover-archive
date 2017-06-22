from PyQt5.QtCore import QObject

from controller.arm_controller import ArmController
from controller.camera_controller import CameraController
from controller.dcdc_controller import DCDC_Controller
from controller.drive_controller import DriveController
from controller.hsv_controller import HSV_Controller
from controller.joystick.joystick_controller import JoystickController
from controller.navigation_controller import NavigationController
from controller.pan_tilt_controller import PanTiltController
from controller.science_controller import ScienceController


class MainController(QObject):

    """!@brief Main controller for the application

    Responsible for linking the different lower level controllers together
    and to their views. This should be the main router for the signals and
    slot mechanisms.
    """

    def __init__(self, main_view, map_enable=False, parent=None):
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
        ## Arm controller, links to ROS arm systems
        self.arm_controller = ArmController(main_view.arm_view, self)
        self.joystick_master.addController("Arm", self.arm_controller)
        ## Navigation controller, subscribes to the ahrs publisher
        self.navigation_controller = NavigationController(self)

        self.hsv_control = HSV_Controller(self)
        main_view.hsv_values.updateParams.connect(self.hsv_control.sendNewConfig)

        self.science_controller = ScienceController(self)
        self.joystick_master.addController("Science", self.science_controller)
        self.pan_cam = PanTiltController(self)
        self.joystick_master.addController("Pan Tilt", self.pan_cam)

        ## Camera controller
        self.camera_controller = CameraController(self)
        self.camera_controller.add_screen(main_view.nav_screen.left_wheel)
        self.camera_controller.add_screen(main_view.nav_screen.right_wheel)
        self.camera_controller.add_screen(main_view.nav_screen.bottom_cam)


        self.drive_controller.wheelStatusUpdate.connect(main_view.drive_view.updateMotorStatus)
        self.drive_controller.forceControlsUpdate.connect(main_view.drive_view.displayDriveSettings)
        main_view.drive_view.controlsUpdated.connect(self.drive_controller.setDriveSetting)
        main_view.nav_mode.activateHCIMode.connect(self.drive_controller.activateHCIMode)
        main_view.nav_mode.activateAutonomousMode.connect(self.drive_controller.activateAutonomousMode)
        self.arm_controller.enableMotors.connect(main_view.drive_view.show_motor_enable)

        self.navigation_controller.updatePitch.connect(main_view.navigation_view.handle_new_pitch)
        self.navigation_controller.updateRoll.connect(main_view.navigation_view.handle_new_roll)
        self.navigation_controller.updateYaw.connect(main_view.navigation_view.handle_new_yaw)
        self.navigation_controller.updateAttitude.connect(main_view.pose_display.update_pose)

        if map_enable:
            self.navigation_controller.updatePosition.connect(main_view.map.add_point)
            main_view.map.coord_widget.createWaypoint.connect(main_view.map.add_waypoint)

        nuc_controller = DCDC_Controller(self)
        nuc_controller.updateIv.connect(main_view.power_info.update_iv)
        nuc_controller.updateIc.connect(main_view.power_info.update_ic)
        nuc_controller.updateOv.connect(main_view.power_info.update_ov)
        nuc_controller.updateOc.connect(main_view.power_info.update_oc)
        nuc_controller.updateOp.connect(main_view.power_info.update_op)
        nuc_controller.updateTemp.connect(main_view.power_info.update_temp)

        self.science_controller.carriageEncoderUpdate.connect(main_view.science_info.update_carriage_position)
        self.science_controller.drillEncoderUpdate.connect(main_view.science_info.update_drill_position)
        self.science_controller.drillSpeedUpdate.connect(main_view.science_info.update_drill_speed)
        self.science_controller.probeSpeedUpdate.connect(main_view.science_info.update_probe_speed)
        self.science_controller.carriagePositionUpdate.connect(main_view.science_info.update_carriage_speed)
        self.science_controller.humidityUpdate.connect(main_view.science_info.update_humidity)
        self.science_controller.temperatureUpdate.connect(main_view.science_info.update_temperature)
        self.science_controller.windUpdate.connect(main_view.science_info.update_wind)

    def __del__(self):
        self.joystick_master.stop()
