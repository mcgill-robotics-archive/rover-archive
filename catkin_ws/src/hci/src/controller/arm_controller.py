import rospy
import tf
from PyQt5.QtCore import pyqtSlot
from geometry_msgs.msg import Pose
from tf.transformations import quaternion_multiply

from controller.joystick.joystick_base import JoystickBase
from model.joystick_data import JoystickData
from views.arm.arm_view import ArmControlMode, DOF, Joint
from arm_control.msg import JointVelocities


class ArmController(JoystickBase):
    def __init__(self, arm_view=None, parent=None):
        super(ArmController, self).__init__(parent)
        self.arm_view = arm_view

        self._mode = ArmControlMode.CLOSED_LOOP
        self._dof = DOF.ORIENTATION
        self._joint = Joint.BASE

        self.velocity_publisher = rospy.Publisher("joint_velocity", JointVelocities, queue_size=1)
        self.position_publisher = rospy.Publisher("end_effector_pose", Pose, queue_size=1)

        self.end_pose = Pose()

    @pyqtSlot(JoystickData)
    def handle_joystick_data(self, data):
        if data.b8:
            self.arm_view.set_closed_loop()
        elif data.b7:
            self.arm_view.set_open_loop()

        if self._mode == ArmControlMode.OPEN_LOOP:
            message = JointVelocities()
            if self._joint == Joint.BASE:
                message.base_yaw = data.a3
                message.base_pitch = data.a2
            elif self._joint == Joint.DIFF1:
                message.diff_1_pitch = data.a2
                message.diff_1_roll = data.a1
            elif self._joint == Joint.DIFF2:
                message.diff_2_pitch = data.a2
                message.diff_2_roll = data.a1
            elif self._joint == Joint.END:
                message.end_effector = data.a2

            self.velocity_publisher.publish(message)

        elif self._mode == ArmControlMode.CLOSED_LOOP:
            if self._dof == DOF.POSITION:
                self.end_pose.position.x += data.a1
                self.end_pose.position.y += data.a2
                self.end_pose.position.z += data.a3

            elif self._dof == DOF.ORIENTATION:
                quaternion = tf.transformations.quaternion_from_euler(data.a2, data.a1, data.a3)
                quaternion = quaternion_multiply((1, 0, 0, 0), (1, 0, 0, 0))
                self.end_pose.orientation.x = quaternion[0]

            self.position_publisher.publish(self.end_pose)
