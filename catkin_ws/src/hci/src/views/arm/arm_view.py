from enum import Enum

import sys

from PyQt5.QtCore import QObject
from PyQt5.QtCore import pyqtSignal
from PyQt5.QtWidgets import QApplication
from PyQt5.QtWidgets import QFrame
from PyQt5.QtWidgets import QGridLayout
from PyQt5.QtWidgets import QHBoxLayout
from PyQt5.QtWidgets import QPushButton
from PyQt5.QtWidgets import QRadioButton
from PyQt5.QtWidgets import QVBoxLayout
from PyQt5.QtWidgets import QWidget


class ArmControlMode(Enum):
    OPEN_LOOP = 1
    CLOSED_LOOP = 2


class Joint(Enum):
    BASE = 1
    DIFF1 = 2
    DIFF2 = 3
    END = 4


class DOF(Enum):
    POSITION = 1
    ORIENTATION = 2


class ArmView(QWidget):

    controlJoint = pyqtSignal(int)
    controlDOF = pyqtSignal(int)
    controlMode = pyqtSignal(int)

    def __init__(self, parent=None):
        super(ArmView, self).__init__(parent)

        v1 = QVBoxLayout()
        h1 = QHBoxLayout()
        g1 = QGridLayout()
        h3 = QHBoxLayout()

        line_2 = QFrame(self)
        line_2.setFrameShape(QFrame.HLine)
        line_2.setFrameShadow(QFrame.Sunken)
        line_1 = QFrame(self)
        line_1.setFrameShape(QFrame.HLine)
        line_1.setFrameShadow(QFrame.Sunken)

        self.open_loop_button = QPushButton("Open Loop", self)
        self.closed_loop_button = QPushButton("Closed Loop", self)
        self.open_loop_button.setCheckable(True)
        self.closed_loop_button.setCheckable(True)
        h1.addWidget(self.open_loop_button)
        h1.addWidget(self.closed_loop_button)

        joint_frame = QFrame(self)
        self.base_joint_button = QRadioButton("Base", joint_frame)
        self.diff1_joint_button = QRadioButton("Diff 1", joint_frame)
        self.diff2_joint_button = QRadioButton("Diff 2", joint_frame)
        self.end_joint_button = QRadioButton("End Eff", joint_frame)
        g1.addWidget(self.base_joint_button, 0, 0)
        g1.addWidget(self.diff1_joint_button, 1, 0)
        g1.addWidget(self.diff2_joint_button, 0, 1)
        g1.addWidget(self.end_joint_button, 1, 1)
        joint_frame.setLayout(g1)

        dof_frame = QFrame(self)
        self.position_button = QRadioButton("Position", dof_frame)
        self.orientation_button = QRadioButton("Orientation", dof_frame)
        h3.addWidget(self.position_button)
        h3.addWidget(self.orientation_button)
        dof_frame.setLayout(h3)

        v1.addItem(h1)
        v1.addWidget(line_1)
        v1.addWidget(joint_frame)
        v1.addWidget(line_2)
        v1.addWidget(dof_frame)
        self.setLayout(v1)

        self.open_loop_button.clicked.connect(self.set_open_loop)
        self.closed_loop_button.clicked.connect(self.set_closed_loop)

        self.base_joint_button.clicked.connect(self.set_base_controlled)
        self.diff1_joint_button.clicked.connect(self.set_diff1_controlled)
        self.diff2_joint_button.clicked.connect(self.set_diff2_controlled)
        self.end_joint_button.clicked.connect(self.set_end_controlled)

        self.position_button.clicked.connect(self.set_position_controlled)
        self.orientation_button.clicked.connect(self.set_orientation_controlled)

    def set_open_loop(self):
        self._set_operation_mode(ArmControlMode.OPEN_LOOP)

    def set_closed_loop(self):
        self._set_operation_mode(ArmControlMode.CLOSED_LOOP)

    def set_base_controlled(self):
        self._set_joint_controlled(Joint.BASE)

    def set_diff1_controlled(self):
        self._set_joint_controlled(Joint.DIFF1)

    def set_diff2_controlled(self):
        self._set_joint_controlled(Joint.DIFF2)

    def set_end_controlled(self):
        self._set_joint_controlled(Joint.END)

    def set_position_controlled(self):
        self._set_dof_controlled(DOF.POSITION)

    def set_orientation_controlled(self):
        self._set_dof_controlled(DOF.ORIENTATION)

    def _set_operation_mode(self, mode, ):
        if mode == ArmControlMode.CLOSED_LOOP:
            self.open_loop_button.setChecked(False)
            self.closed_loop_button.setChecked(True)
            self._enable_closed(True)
        elif mode == ArmControlMode.OPEN_LOOP:
            self.open_loop_button.setChecked(True)
            self.closed_loop_button.setChecked(False)
            self._enable_closed(False)
        else:
            pass
        self.controlMode.emit(mode)

    def _enable_closed(self, value):
        self.position_button.setEnabled(value)
        self.orientation_button.setEnabled(value)
        self.base_joint_button.setEnabled(not value)
        self.diff1_joint_button.setEnabled(not value)
        self.diff2_joint_button.setEnabled(not value)
        self.end_joint_button.setEnabled(not value)

    def _set_joint_controlled(self, joint):
        print joint
        self.controlJoint.emit(joint)
        self.set_open_loop()
        if joint == Joint.BASE:
            self.base_joint_button.setChecked(True)
        elif joint == Joint.DIFF1:
            self.diff1_joint_button.setChecked(True)
        elif joint == Joint.DIFF2:
            self.diff2_joint_button.setChecked(True)
        elif joint == Joint.END:
            self.end_joint_button.setChecked(True)

    def _set_dof_controlled(self, dof):
        print dof
        self.controlDOF.emit(dof)
        self.set_closed_loop()
        if dof == DOF.POSITION:
            self.position_button.setChecked(True)
        elif dof == DOF.ORIENTATION:
            self.orientation_button.setChecked(True)


if __name__ == '__main__':
    APP = QApplication(sys.argv)
    WIN = ArmView()
    WIN.show()
    WIN.set_base_controlled()
    WIN.set_orientation_controlled()
    sys.exit(APP.exec_())
