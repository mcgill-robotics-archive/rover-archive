"""!@brief Textual display of the position and orientation of the robot

"""
import tf
from PyQt5.QtCore import pyqtSignal, pyqtSlot
from PyQt5.QtGui import QFont
from PyQt5.QtWidgets import QApplication
from PyQt5.QtWidgets import QHBoxLayout
from PyQt5.QtWidgets import QLabel
from PyQt5.QtWidgets import QVBoxLayout
from PyQt5.QtWidgets import QWidget

from geometry_msgs.msg import Pose

import sys

from utilities import format_euler_angle, format_dms


class PoseDisplay(QWidget):
    """!@brief GUI class to textually display an orientation and position

    Samples come in as geometry_msgs/Pose messages directly.
    """

    ## Emit when the pose is changed
    poseUpdated = pyqtSignal(Pose)

    def __init__(self, parent=None):
        """!@brief Constructor inits and places the objects

        @param parent Qt object hierarchy
        @param self Python object pointer
        """
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

        self._latitude = QLabel(self)
        self._longitude = QLabel(self)

        hbox1.addWidget(lat_t)
        hbox1.addWidget(self._latitude)

        hbox2.addWidget(lon_t)
        hbox2.addWidget(self._longitude)

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

        self._pitch = QLabel(self)
        self._roll = QLabel(self)
        self._yaw = QLabel(self)

        hbox4.addWidget(pit_t)
        hbox4.addWidget(self._pitch)
        hbox4.addWidget(rol_t)
        hbox4.addWidget(self._roll)
        hbox4.addWidget(yaw_t)
        hbox4.addWidget(self._yaw)

        vbox1.addItem(hbox3)
        vbox1.addItem(hbox4)

        self.setLayout(vbox1)
        self.update_pose()

    @pyqtSlot(Pose)
    def update_pose(self, pose = Pose()):
        """!@brief Slot to update the pose displayed

        Extract quaternion and convert to euler for display.
        Extract GPS position and display

        @param pose The new pose to display
        @param self Python object pointer
        """
        quaternion = (
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )
        euler = tf.transformations.euler_from_quaternion(quaternion)

        self._latitude.setText(format_dms(pose.position.y))
        self._longitude.setText(format_dms(pose.position.x))

        self._roll.setText(format_euler_angle(euler[0]))
        self._pitch.setText(format_euler_angle(euler[1]))
        self._yaw.setText(format_euler_angle(euler[2]))
        self.poseUpdated.emit(pose)


if __name__ == "__main__":
    app = QApplication(sys.argv)
    ui = PoseDisplay()
    ui.show()
    sys.exit(app.exec_())
