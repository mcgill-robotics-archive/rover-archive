import rospy
import tf
from PyQt5.QtCore import pyqtSlot, pyqtSignal
from geometry_msgs.msg import Pose, PoseStamped
from tf.transformations import quaternion_multiply

from controller.joystick.joystick_base import JoystickBase
from model.joystick_data import JoystickData
from views.arm.arm_view import ArmControlMode, DOF, Joint
from arm_control.msg import JointVelocities


class ArmController(JoystickBase):
    enableMotors = pyqtSignal(int)

    def __init__(self, arm_view=None, parent=None):
        super(ArmController, self).__init__(parent)

        ## Internal pointer to linked view element
        self.arm_view = arm_view

        self._mode = ArmControlMode.OPEN_LOOP
        self._dof = DOF.ORIENTATION
        self._joint = Joint.BASE
        self.arm_view.set_orientation_controlled()
        self.arm_view.set_base_controlled()

        ## ROS publisher for velocity control
        self.velocity_publisher = rospy.Publisher("joint_velocity", JointVelocities, queue_size=1)
        ## ROS publisher for position control
        self.position_publisher = rospy.Publisher("end_effector_pose", Pose, queue_size=1)

        self._end_pose = Pose()

        self.arm_view.controlMode.connect(self._update_control_mode)
        self.arm_view.controlDOF.connect(self._update_dof)
        self.arm_view.controlJoint.connect(self._update_joint)

        self._pitch = 0
        self._roll = 0
        self._yaw = 0

    @pyqtSlot(JoystickData)
    def handle_joystick_data(self, data):
        """!@brief Handle joystick data concrete implementation for arm
        module. React to key and axis input

        @param data
        @param self Python object pointer
        """
        self.enableMotors.emit(data.b1)

        if self._mode == ArmControlMode.OPEN_LOOP:
            if data.b7:
                # change to base motor
                self.arm_view.set_base_controlled()
                pass
            elif data.b8:
                self.arm_view.set_diff2_controlled()
            elif data.b9:
                self.arm_view.set_diff1_controlled()
            elif data.b10:
                self.arm_view.set_end_controlled()

            message = JointVelocities()
            if data.b1:
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
                self._end_pose.position.x += data.a1
                self._end_pose.position.y += data.a2
                self._end_pose.position.z += data.a3

            elif self._dof == DOF.ORIENTATION:
                self._pitch += data.a1
                self._roll += data.a2
                self._yaw += data.a3

                quaternion = tf.transformations.quaternion_from_euler(self._roll, self._pitch, self._yaw)
                self._end_pose.orientation.x = quaternion[0]
                self._end_pose.orientation.y = quaternion[1]
                self._end_pose.orientation.z = quaternion[2]
                self._end_pose.orientation.w = quaternion[3]

            self.position_publisher.publish(self._end_pose)

    def _update_control_mode(self, mode):
        print mode
        self._mode = mode

    def _update_joint(self, joint):
        print joint
        self._joint = joint

    def _update_dof(self, dof):
        print dof
        self._dof = dof

