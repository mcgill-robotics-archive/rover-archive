# -*- coding: utf-8 -*-

""" Human Computer Interaction. """
from PyQt5.QtWidgets import QApplication
from PyQt5.QtWidgets import QHBoxLayout
from PyQt5.QtWidgets import QVBoxLayout
from PyQt5.QtWidgets import QWidget
from geometry_msgs.msg import Pose

from views.attitude_indicator.attitude import QAttitude
from views.attitude_indicator.compass import QCompass
from views.map.waypoint_entry import WaypointPad
from views.drive.drive_control import SteeringMode
from views.science.science_indicator import DrillStatus
from views.map.pose_display import PoseDisplay

import sys

__author__ = "David Lavoie-Boutin"
__version__ = "0.2.0"


def run():
    app = QApplication(sys.argv)
    window = QWidget()
    steering = SteeringMode(window)
    drill = DrillStatus(window)
    waypoint = WaypointPad(window)
    pose = PoseDisplay(window)
    lay = QHBoxLayout()
    layV = QVBoxLayout()
    hbox2 = QHBoxLayout()
    compass = QCompass(window)
    atti = QAttitude(window)
    hbox2.addWidget(compass)
    hbox2.addWidget(atti)
    lay.addWidget(waypoint)
    lay.addWidget(steering)
    lay.addWidget(drill)
    layV.addItem(lay)
    layV.addWidget(pose)
    layV.addItem(hbox2)
    window.setLayout(layV)
    window.show()

    def retransmit(lon, lat):
        pose_msg = Pose()
        pose_msg.position.x = lon
        pose_msg.position.y = lat
        pose.update_pose(pose_msg)

    waypoint.createWaypoint.connect(retransmit)
    sys.exit(app.exec_())


if __name__ == '__main__':
    run()
