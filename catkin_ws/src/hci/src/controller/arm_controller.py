import rospy
import tf
from PyQt5.QtCore import pyqtSignal
from PyQt5.QtCore import pyqtSlot
from geometry_msgs.msg import Pose, PoseStamped
from tf.transformations import quaternion_multiply

from controller.joystick.joystick_base import JoystickBase
from model.joystick_data import JoystickData
from views.arm.arm_view import ArmControlMode, DOF, Joint
from arm_control.msg import JointVelocities


class ArmController(JoystickBase):
    updateMotorEnable = pyqtSignal(int)
    def __init__(self, arm_view=None, parent=None):
        super(ArmController, self).__init__(parent)
        self.arm_view = arm_view

        self._mode = ArmControlMode.OPEN_LOOP
        self._dof = DOF.ORIENTATION
        self._joint = Joint.BASE
        self.arm_view.set_orientation_controlled()
        self.arm_view.set_base_controlled()

        self.velocity_publisher = rospy.Publisher("arm_velocities", JointVelocities, queue_size=1)
        self.position_publisher = rospy.Publisher("end_effector_pose", Pose, queue_size=1)

        self.end_pose = Pose()

        self.arm_view.controlMode.connect(self.update_control_mode)
        self.arm_view.controlDOF.connect(self.update_dof)
        self.arm_view.controlJoint.connect(self.update_joint)

        self.pitch = 0
        self.roll = 0
        self.yaw = 0
        self.motor_enable = 0

    @pyqtSlot(JoystickData)
    def handle_joystick_data(self, data):

        if self.motor_enable != data.b1:
            self.motor_enable = data.b1
            self.updateMotorEnable.emit(self.motor_enable)


        if data.b8:
            self.arm_view.set_closed_loop()
        elif data.b7:
            self.arm_view.set_open_loop()

        # Check the mode for open loop or closed loop.
        if self._mode == ArmControlMode.OPEN_LOOP:
            message = JointVelocities()
            if self._joint == Joint.BASE:
                message.base_yaw = data.a3 * self.motor_enable * 100
                message.base_pitch = data.a2 * self.motor_enable * 100
            elif self._joint == Joint.DIFF1:
                message.diff_1_pitch = data.a2 * self.motor_enable * 100
                message.diff_1_roll = data.a1 * self.motor_enable * 100
                # message.base_pitch = data.a2 * -100
            elif self._joint == Joint.DIFF2:
                message.diff_2_pitch = data.a2 * self.motor_enable * 100
                message.diff_2_roll = data.a1 * self.motor_enable * 100
            elif self._joint == Joint.END:
                message.end_effector = data.a2 * self.motor_enable * 100

            self.velocity_publisher.publish(message)

        elif self._mode == ArmControlMode.CLOSED_LOOP and self.motor_enable:
            if self._dof == DOF.POSITION:
                self.end_pose.position.x += data.a1
                self.end_pose.position.y += data.a2
                self.end_pose.position.z += data.a3

            elif self._dof == DOF.ORIENTATION:
                self.pitch += data.a1
                self.roll += data.a2
                self.yaw += data.a3

                quaternion = tf.transformations.quaternion_from_euler(self.roll, self.pitch, self.yaw)
                self.end_pose.orientation.x = quaternion[0]
                self.end_pose.orientation.y = quaternion[1]
                self.end_pose.orientation.z = quaternion[2]
                self.end_pose.orientation.w = quaternion[3]

            self.position_publisher.publish(self.end_pose)

    def update_control_mode(self, mode):
        print mode
        self._mode = mode

    def update_joint(self, joint):
        print joint
        self._joint = joint

    def update_dof(self, dof):
        print dof
        self._dof = dof
