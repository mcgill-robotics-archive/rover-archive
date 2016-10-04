#!/usr/bin/env python

import sys
from PyQt4 import QtGui, QtCore

from PyQt4.QtCore import QObject
from PyQt4.QtCore import pyqtSignal
from PyQt4.QtGui import QCheckBox
from PyQt4.QtGui import QGridLayout
from PyQt4.QtGui import QHBoxLayout
from PyQt4.QtGui import QRadioButton
from PyQt4.QtGui import QVBoxLayout
from PyQt4.QtGui import QWidget


class DriveSettings(object):
    def __init__(self, motor_enable = False, ackerman_steering = False, point_steering = False,translatory_steering = False):
        self.motor_enable = motor_enable
        self.ackerman_steering = ackerman_steering
        self.point_steering = point_steering
        self.translatory_steering = translatory_steering

class SteeringMode(QWidget):
    driveSettingsChanged = pyqtSignal(DriveSettings, name="changeDriveSettings")

    def __init__(self, parent=None):
        super(SteeringMode, self).__init__(parent)

        self.driveSettings = DriveSettings()

        vbox1 = QVBoxLayout()
        vbox1.setContentsMargins(0, 0, 0, 0)
        hbox1 = QHBoxLayout()
        hbox1.setContentsMargins(0, 0, 0, 0)
        grid1 = QGridLayout()
        grid1.setContentsMargins(0, 0, 0, 0)

        self.ackerman = QRadioButton(self)
        self.ackerman.setText("Ackerman Steering")
        self.pointsteer = QRadioButton(self)
        self.pointsteer.setText("Point Steering")
        self.translate = QRadioButton(self)
        self.translate.setText("Translation")

        self.enable = QCheckBox()
        self.enable.setText("Enable Motors")

        vbox1.addWidget(self.ackerman)
        vbox1.addWidget(self.pointsteer)
        vbox1.addWidget(self.translate)
        hbox1.addWidget(self.enable)
        hbox1.addItem(vbox1)

        self.setLayout(hbox1)

        QObject.connect(self.enable, QtCore.SIGNAL("clicked()"), self.handle_enable)
        QObject.connect(self.ackerman, QtCore.SIGNAL("clicked()"), self.handle_steerimg_mode)
        QObject.connect(self.pointsteer, QtCore.SIGNAL("clicked()"), self.handle_steerimg_mode)
        QObject.connect(self.translate, QtCore.SIGNAL("clicked()"), self.handle_steerimg_mode)

    def handle_enable(self):
        if self.enable.isChecked():
            self.driveSettings.motor_enable = True
        else:
            self.driveSettings.motor_enable = False

        self.driveSettingsChanged.emit(self.driveSettings)

    def handle_steerimg_mode(self):
        if self.ackerman.isChecked():
            self.driveSettings.ackerman_steering = True
            self.driveSettings.point_steering = False
            self.driveSettings.translatory_steering = False
        elif self.pointsteer.isChecked():
            self.driveSettings.ackerman_steering = False
            self.driveSettings.point_steering = True
            self.driveSettings.translatory_steering = False
        elif self.translate.isChecked():
            self.driveSettings.ackerman_steering = False
            self.driveSettings.point_steering = False
            self.driveSettings.translatory_steering = True

        self.driveSettingsChanged.emit(self.driveSettings)

    def update_motor_enable(self, motor_enable = False):
        self.driveSettings.motor_enable = motor_enable
        self.enable.setChecked(motor_enable)
        self.driveSettingsChanged.emit(self.driveSettings)

    def update_steering(self, ackerman_steering = False, point_steering = False,translatory_steering = False):
        self.ackerman.setChecked(ackerman_steering)
        self.pointsteer.setChecked(point_steering)
        self.translate.setChecked(translatory_steering)

        self.driveSettings.ackerman_steering = ackerman_steering
        self.driveSettings.point_steering = point_steering
        self.driveSettings.translatory_steering = translatory_steering

        self.driveSettingsChanged.emit(self.driveSettings)

if __name__ == "__main__":
    from PyQt4.QtCore import pyqtSlot

    class DriveWindowTest(QWidget):
        def __init__(self):
            super(DriveWindowTest, self).__init__()
            self.ui = SteeringMode(self)
            self.ui.show()
            self.ui.driveSettingsChanged.connect(self.print_status)

        @pyqtSlot(DriveSettings)
        def print_status(self, drive_setting):
            if drive_setting.ackerman_steering:
                stering = "ackerman"
            elif drive_setting.point_steering:
                stering = "point"
            elif drive_setting.translatory_steering:
                stering = "translatory"
            else:
                stering = "none"

            print "Status: enable {0}, steering: {1}".format(drive_setting.motor_enable, stering)

    app = QtGui.QApplication(sys.argv)
    ui = DriveWindowTest()
    ui.show()
    ui.ui.update_motor_enable(True)
    ui.ui.update_steering(translatory_steering=True)
    sys.exit(app.exec_())