from PyQt4 import QtCore
from PyQt4 import QtGui

import sys

from PyQt4.QtCore import QObject
from PyQt4.QtCore import pyqtSignal, pyqtSlot
from PyQt4.QtGui import QGridLayout
from PyQt4.QtGui import QHBoxLayout
from PyQt4.QtGui import QLabel
from PyQt4.QtGui import QPushButton
from PyQt4.QtGui import QTabWidget
from PyQt4.QtGui import QVBoxLayout
from PyQt4.QtGui import QWidget


class WaypointPad(QWidget):
    createWaypoint = pyqtSignal(float, float)

    def __init__(self, parent=None):
        super(WaypointPad, self).__init__(parent)

        dms_tab = QWidget()
        grid1 = QGridLayout()
        grid2 = QGridLayout()

        vbox1 = QVBoxLayout()
        vbox2 = QVBoxLayout()

        hbox1 = QHBoxLayout()

        lon_t = QLabel("Longitude (X)", dms_tab)
        lat_t = QLabel("Latitude (Y)", dms_tab)

        self.lon_sign = QtGui.QComboBox(dms_tab)
        self.lon_sign.addItem("E")
        self.lon_sign.addItem("W")
        self.lon_deg = QtGui.QSpinBox(dms_tab)
        self.lon_min = QtGui.QSpinBox(dms_tab)
        self.lon_sec = QtGui.QDoubleSpinBox(dms_tab)

        grid1.addWidget(self.lon_sign, 0, 0, 1, 1)
        grid1.addWidget(self.lon_deg, 0, 1, 1, 1)
        grid1.addWidget(self.lon_min, 1, 0, 1, 1)
        grid1.addWidget(self.lon_sec, 1, 1, 1, 1)
        vbox1.addWidget(lon_t)
        vbox1.addItem(grid1)

        self.lat_sign = QtGui.QComboBox(dms_tab)
        self.lat_sign.addItem(("N"))
        self.lat_sign.addItem(("S"))
        self.lat_deg = QtGui.QSpinBox(dms_tab)
        self.lat_min = QtGui.QSpinBox(dms_tab)
        self.lat_sec = QtGui.QDoubleSpinBox(dms_tab)

        grid2.addWidget(self.lat_sign, 0, 0, 1, 1)
        grid2.addWidget(self.lat_deg, 0, 1, 1, 1)
        grid2.addWidget(self.lat_min, 1, 0, 1, 1)
        grid2.addWidget(self.lat_sec, 1, 1, 1, 1)
        vbox2.addWidget(lat_t)
        vbox2.addItem(grid2)

        self.add_dms_button = QPushButton("Add Waypoint", dms_tab)

        hbox1.addItem(vbox1)
        hbox1.addItem(vbox2)
        hbox1.addWidget(self.add_dms_button)
        dms_tab.setLayout(hbox1)

        dd_tab = QWidget()

        vbox3 = QVBoxLayout()
        vbox4 = QVBoxLayout()

        dd_lon_t = QLabel("Longitude (x)", dd_tab)
        dd_lat_t = QLabel("Latitude (y)", dd_tab)
        self.dd_lat = QtGui.QDoubleSpinBox(dd_tab)
        self.dd_lon = QtGui.QDoubleSpinBox(dd_tab)

        self.add_dd_button = QPushButton("Add Waypoint", dd_tab)

        vbox3.addWidget(dd_lon_t)
        vbox3.addWidget(self.dd_lon)

        vbox4.addWidget(dd_lat_t)
        vbox4.addWidget(self.dd_lat)

        hbox3 = QHBoxLayout()
        hbox3.addItem(vbox3)
        hbox3.addItem(vbox4)
        hbox3.addWidget(self.add_dd_button)

        dd_tab.setLayout(hbox3)

        tab_widget = QTabWidget(self)
        tab_widget.addTab(dms_tab, "DMS")
        tab_widget.addTab(dd_tab, "DD")

        hbox2 = QHBoxLayout()
        hbox2.addWidget(tab_widget)
        self.setLayout(hbox2)

        QObject.connect(self.add_dms_button, QtCore.SIGNAL("clicked()"), self.create_dms_waypoint)
        QObject.connect(self.add_dd_button, QtCore.SIGNAL("clicked()"), self.create_dd_waypoint)

    def create_dms_waypoint(self):
        longitude = self.lon_deg.value() + self.lon_min.value() / 60.0 + self.lon_sec.value() / 3600.0
        latitude = self.lat_deg.value() + self.lat_min.value() / 60.0 + self.lat_sec.value() / 3600.0

        if self.lat_sign.currentIndex() == 1:
            latitude = - latitude

        if self.lon_sign.currentIndex() == 1:
            longitude = - longitude

        self.createWaypoint.emit(longitude, latitude)
        pass
    
    def create_dd_waypoint(self):
        self.createWaypoint.emit(self.dd_lon.value(), self.dd_lat.value())
        pass
    
    
if __name__ == "__main__":

    class Tester(QWidget):
        def __init__(self):
            super(Tester, self).__init__()
            self.ui = WaypointPad(self)
            self.ui.show()
            self.ui.createWaypoint.connect(self.handle_new_point)

        @pyqtSlot(float, float)
        def handle_new_point(self, x, y):
            print "X: {0}, Y: {1}".format(x, y)

    app = QtGui.QApplication(sys.argv)
    ui = Tester()
    ui.show()
    sys.exit(app.exec_())
