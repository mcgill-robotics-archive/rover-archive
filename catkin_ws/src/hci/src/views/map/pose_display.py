from PyQt4 import QtGui

import tf
from PyQt4.QtCore import pyqtSignal, pyqtSlot
from PyQt4.QtGui import QFont
from PyQt4.QtGui import QHBoxLayout
from PyQt4.QtGui import QLabel
from PyQt4.QtGui import QVBoxLayout
from PyQt4.QtGui import QWidget

from geometry_msgs.msg import Pose

import sys

from hci.src.utilities import format_euler_angle, format_dms


class PoseDisplay(QWidget):
    poseUpdated = pyqtSignal(Pose)

    def __init__(self, parent=None):
        super(PoseDisplay, self).__init__(parent)

        hbox1 = QHBoxLayout()
        hbox2 = QHBoxLayout()
        hbox3 = QHBoxLayout()
        hbox4 = QHBoxLayout()
        hbox5 = QHBoxLayout()
        vbox1 = QVBoxLayout()

        title_font = QFont()
        title_font.setPointSize(15)
        title_font.setBold(True)
        title_font.setWeight(75)

        lat_t = QLabel("Lat (N/S)", self)
        lon_t = QLabel("Lon (E/W)", self)
        lat_t.setFont(title_font)
        lon_t.setFont(title_font)
        lat_t.setFixedWidth(90)
        lon_t.setFixedWidth(100)

        self.latitude = QLabel(self)
        self.longitude = QLabel(self)

        hbox1.addWidget(lat_t)
        hbox1.addWidget(self.latitude)

        hbox2.addWidget(lon_t)
        hbox2.addWidget(self.longitude)

        hbox3.addItem(hbox1)
        hbox3.addItem(hbox2)

        pit_t = QLabel("Pitch", self)
        rol_t = QLabel("Roll", self)
        yaw_t = QLabel("Yaw", self)
        pit_t.setFont(title_font)
        rol_t.setFont(title_font)
        yaw_t.setFont(title_font)
        pit_t.setFixedWidth(60)
        rol_t.setFixedWidth(60)
        yaw_t.setFixedWidth(60)

        self.pitch = QLabel(self)
        self.roll = QLabel(self)
        self.yaw = QLabel(self)

        hbox4.addWidget(pit_t)
        hbox4.addWidget(self.pitch)
        hbox4.addWidget(rol_t)
        hbox4.addWidget(self.roll)
        hbox4.addWidget(yaw_t)
        hbox4.addWidget(self.yaw)

        vbox1.addItem(hbox3)
        vbox1.addItem(hbox4)

        self.setLayout(vbox1)
        self.update_pose()

    @pyqtSlot(Pose)
    def update_pose(self, pose = Pose()):
        quaternion = (
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )
        euler = tf.transformations.euler_from_quaternion(quaternion)

        self.latitude.setText(format_dms(pose.position.y))
        self.longitude.setText(format_dms(pose.position.x))

        self.roll.setText(format_euler_angle(euler[0]))
        self.pitch.setText(format_euler_angle(euler[1]))
        self.yaw.setText(format_euler_angle(euler[2]))
        self.poseUpdated.emit(pose)


if __name__ == "__main__":
    app = QtGui.QApplication(sys.argv)
    ui = PoseDisplay()
    ui.show()
    sys.exit(app.exec_())
