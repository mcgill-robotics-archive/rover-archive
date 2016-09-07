#!/usr/bin/env python

import sys
from PyQt4 import QtGui, QtCore

from PyQt4.QtCore import pyqtSignal
from PyQt4.QtCore import QObject
from PyQt4.QtGui import QCheckBox
from PyQt4.QtGui import QHBoxLayout
from PyQt4.QtGui import QLabel
from PyQt4.QtGui import QPushButton
from PyQt4.QtGui import QVBoxLayout
from PyQt4.QtGui import QWidget

from hci.src.utilities import *


class DrillStatus(QWidget):
    openRockContainter = pyqtSignal(name="openRockContainter")
    closeRockContainter = pyqtSignal(name="closeRockContainter")

    openSoilContainter = pyqtSignal(name="openSoilContainter")
    closeSoilContainter = pyqtSignal(name="closeSoilContainter")
    getSensors = pyqtSignal()

    reverseDrill = pyqtSignal()
    forwardDrill = pyqtSignal()
    offDrill = pyqtSignal()

    def __init__(self, parent=None):
        super(DrillStatus, self).__init__(parent)

        hbox1 = QHBoxLayout()
        hbox1.setContentsMargins(0, 0, 0, 0)
        hbox2 = QHBoxLayout()
        hbox2.setContentsMargins(0, 0, 0, 0)
        hbox3 = QHBoxLayout()
        hbox3.setContentsMargins(0, 0, 0, 0)
        hbox4 = QHBoxLayout()
        hbox4.setContentsMargins(0, 0, 0, 0)
        hbox5 = QHBoxLayout()
        hbox5.setContentsMargins(0, 0, 0, 0)
        vbox1 = QVBoxLayout()
        vbox1.setContentsMargins(0, 0, 0, 0)

        label1 = QLabel(self)
        label1.setText("Limit Switch Up")
        label2 = QLabel(self)
        label2.setText("Limit Switch Down")
        self.ls_up_status = QLabel(self)
        self.limit_switch_up_off()
        self.ls_down_status = QLabel(self)
        self.limit_switch_down_off()

        label3 = QLabel(self)
        label3.setText("Drill status")
        self.drill_status_label = QLabel(self)
        self.drill_off()

        line_2 = QtGui.QFrame(self)
        line_2.setFrameShape(QtGui.QFrame.HLine)
        line_2.setFrameShadow(QtGui.QFrame.Sunken)

        self.rock_checkbox = QCheckBox(self)
        self.rock_checkbox.setText("Rock Container")
        self.rock_status = QLabel(self)

        self.soil_checkbox = QCheckBox()
        self.soil_checkbox.setText("Soil Container")
        self.soil_status = QLabel(self)

        self.closed_rock_container()
        self.closed_soil_container()

        self.drill_forward_button = QtGui.QRadioButton()
        self.drill_forward_button.setText("Drill Forward")
        self.drill_reverse_button = QtGui.QRadioButton()
        self.drill_reverse_button.setText("Drill Reverse")
        self.drill_off_button = QtGui.QRadioButton()
        self.drill_off_button.setText("Drill OFF")
        self.drill_off_button.click()

        line_3 = QtGui.QFrame(self)
        line_3.setFrameShape(QtGui.QFrame.HLine)
        line_3.setFrameShadow(QtGui.QFrame.Sunken)

        self.sensor_button = QPushButton(self)
        self.sensor_button.setText("Get Sensor Data")

        hbox1.addWidget(label1)
        hbox1.addWidget(self.ls_up_status)
        hbox1.addWidget(label2)
        hbox1.addWidget(self.ls_down_status)
        hbox2.addWidget(label3)
        hbox2.addWidget(self.drill_status_label)
        hbox3.addWidget(self.rock_checkbox)
        hbox3.addWidget(self.rock_status)
        hbox4.addWidget(self.soil_checkbox)
        hbox4.addWidget(self.soil_status)
        hbox5.addWidget(self.drill_off_button)
        hbox5.addWidget(self.drill_forward_button)
        hbox5.addWidget(self.drill_reverse_button)
        vbox1.addItem(hbox1)
        vbox1.addItem(hbox2)
        vbox1.addWidget(line_2)
        vbox1.addItem(hbox3)
        vbox1.addItem(hbox4)
        vbox1.addWidget(line_3)
        vbox1.addItem(hbox5)
        vbox1.addWidget(self.sensor_button)
        self.setLayout(vbox1)

        QObject.connect(self.drill_off_button, QtCore.SIGNAL("clicked()"), self.drill_button)
        QObject.connect(self.drill_forward_button, QtCore.SIGNAL("clicked()"), self.drill_button)
        QObject.connect(self.drill_reverse_button, QtCore.SIGNAL("clicked()"), self.drill_button)
        QObject.connect(self.sensor_button, QtCore.SIGNAL("clicked()"), self.send_sensors)
    def drill_reverse(self):
        lbl_bg_red(self.drill_status_label, "Reverse")
        return

    def drill_forward(self):
        lbl_bg_red(self.drill_status_label, "Forward")
        return

    def drill_off(self):
        lbl_bg_grn(self.drill_status_label, "Off")
        return

    def limit_switch_up_on(self):
        lbl_bg_red(self.ls_up_status, "ON")
        return

    def limit_switch_down_on(self):
        lbl_bg_red(self.ls_down_status, "ON")
        return

    def limit_switch_up_off(self):
        lbl_bg_grn(self.ls_up_status, "OFF")
        return

    def limit_switch_down_off(self):
        lbl_bg_grn(self.ls_down_status, "OFF")
        return

    def rock_checkbox_callback(self):
        if self.rock_checkbox.isChecked():
            self.openRockContainter.emit()
        else:
            self.closeRockContainter.emit()

    def soil_checkbox_callback(self):
        if self.soil_checkbox.isChecked():
            self.openSoilContainter.emit()
        else:
            self.closeSoilContainter.emit()

    def opened_soil_container(self):
        lbl_bg_red(self.soil_status, "OPENED")

    def closed_soil_container(self):
        lbl_bg_grn(self.soil_status, "CLOSED")

    def opened_rock_container(self):
        lbl_bg_red(self.rock_status, "OPENED")

    def closed_rock_container(self):
        lbl_bg_grn(self.rock_status, "CLOSED")

    def drill_button(self):
        if self.drill_forward_button.isChecked():
            self.forwardDrill.emit()
        elif self.drill_reverse_button.isChecked():
            self.reverseDrill.emit()
        else:
            self.offDrill.emit()

    def send_sensors(self):
        self.getSensors.emit()

if __name__ == "__main__":
    app = QtGui.QApplication(sys.argv)
    ui = DrillStatus()
    ui.show()
    sys.exit(app.exec_())
