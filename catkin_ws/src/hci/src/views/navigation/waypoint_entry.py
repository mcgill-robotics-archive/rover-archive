"""!@brief Generate coordinates from different formats

Gui module showing prompt for creating waypoints. Includes a test class showing general normal usage

"""
import sys

from PyQt5.QtCore import pyqtSignal, pyqtSlot
from PyQt5.QtWidgets import QApplication
from PyQt5.QtWidgets import QComboBox
from PyQt5.QtWidgets import QDoubleSpinBox
from PyQt5.QtWidgets import QGridLayout
from PyQt5.QtWidgets import QHBoxLayout
from PyQt5.QtWidgets import QLabel
from PyQt5.QtWidgets import QPushButton
from PyQt5.QtWidgets import QSpinBox
from PyQt5.QtWidgets import QTabWidget
from PyQt5.QtWidgets import QVBoxLayout
from PyQt5.QtWidgets import QWidget


class WaypointPad(QWidget):
    """!@brief GUI class for waypoint entry and coordinate generation

    Creates a set of coordinate from texbox, either in DMS or decimal format.
    Created coordinated is signaled using the createWaypoint signal.
    """

    ## Emitted when new coordinate should be added to the map
    createWaypoint = pyqtSignal(float, float)

    def __init__(self, parent=None):
        """!@brief Constructor creates the ui and connects the handelers

        @param parent Qt hierarchical parent
        @param self Python object pointer
        """
        super(WaypointPad, self).__init__(parent)

        dms_tab = QWidget()
        grid1 = QGridLayout()
        grid2 = QGridLayout()

        vbox1 = QVBoxLayout()
        vbox2 = QVBoxLayout()

        hbox1 = QHBoxLayout()

        lon_t = QLabel("Longitude (X)", dms_tab)
        lat_t = QLabel("Latitude (Y)", dms_tab)

        self._lon_sign = QComboBox(dms_tab)
        self._lon_sign.addItem("E")
        self._lon_sign.addItem("W")
        self._lon_deg = QSpinBox(dms_tab)
        self._lon_min = QSpinBox(dms_tab)
        self._lon_sec = QDoubleSpinBox(dms_tab)
        self._lon_sec.setDecimals(10)

        grid1.addWidget(self._lon_sign, 0, 0, 1, 1)
        grid1.addWidget(self._lon_deg, 0, 1, 1, 1)
        grid1.addWidget(self._lon_min, 1, 0, 1, 1)
        grid1.addWidget(self._lon_sec, 1, 1, 1, 1)
        vbox1.addWidget(lon_t)
        vbox1.addItem(grid1)

        self._lat_sign = QComboBox(dms_tab)
        self._lat_sign.addItem(("N"))
        self._lat_sign.addItem(("S"))
        self._lat_deg = QSpinBox(dms_tab)
        self._lat_min = QSpinBox(dms_tab)
        self._lat_sec = QDoubleSpinBox(dms_tab)
        self._lat_sec.setDecimals(10)

        grid2.addWidget(self._lat_sign, 0, 0, 1, 1)
        grid2.addWidget(self._lat_deg, 0, 1, 1, 1)
        grid2.addWidget(self._lat_min, 1, 0, 1, 1)
        grid2.addWidget(self._lat_sec, 1, 1, 1, 1)
        vbox2.addWidget(lat_t)
        vbox2.addItem(grid2)

        self._add_dms_button = QPushButton("Add Waypoint", dms_tab)

        hbox1.addItem(vbox1)
        hbox1.addItem(vbox2)
        hbox1.addWidget(self._add_dms_button)
        dms_tab.setLayout(hbox1)

        dd_tab = QWidget()

        vbox3 = QVBoxLayout()
        vbox4 = QVBoxLayout()

        dd_lon_t = QLabel("Longitude (x)", dd_tab)
        dd_lat_t = QLabel("Latitude (y)", dd_tab)
        self._dd_lat = QDoubleSpinBox(dd_tab)
        self._dd_lon = QDoubleSpinBox(dd_tab)
        self._dd_lat.setDecimals(10)
        self._dd_lon.setDecimals(10)

        self._add_dd_button = QPushButton("Add Waypoint", dd_tab)

        vbox3.addWidget(dd_lon_t)
        vbox3.addWidget(self._dd_lon)

        vbox4.addWidget(dd_lat_t)
        vbox4.addWidget(self._dd_lat)

        hbox3 = QHBoxLayout()
        hbox3.addItem(vbox3)
        hbox3.addItem(vbox4)
        hbox3.addWidget(self._add_dd_button)

        dd_tab.setLayout(hbox3)

        tab_widget = QTabWidget(self)
        tab_widget.addTab(dms_tab, "DMS")
        tab_widget.addTab(dd_tab, "DD")

        hbox2 = QHBoxLayout()
        hbox2.addWidget(tab_widget)
        self.setLayout(hbox2)

        self._add_dms_button.clicked.connect(self._create_dms_waypoint)
        self._add_dd_button.clicked.connect(self._create_dd_waypoint)

    def _create_dms_waypoint(self):
        """!@brief Handle creation of new points from DMS entry

        Called upon button press for new waypoint. Will emit signal with
        waypoint coordinates converted from DMS to decimal format

        @param self Python object pointer
        """
        longitude = self._lon_deg.value() + self._lon_min.value() / 60.0 + self._lon_sec.value() / 3600.0
        latitude = self._lat_deg.value() + self._lat_min.value() / 60.0 + self._lat_sec.value() / 3600.0

        if self._lat_sign.currentIndex() == 1:
            latitude = - latitude

        if self._lon_sign.currentIndex() == 1:
            longitude = - longitude

        self.createWaypoint.emit(longitude, latitude)
        pass

    def _create_dd_waypoint(self):
        """!@brief Created new waypoint

        Emits signal using the decimal coordinates

        @parent self Python object pointer
        """
        self.createWaypoint.emit(self._dd_lon.value(), self._dd_lat.value())
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
            print("X: {0}, Y: {1}".format(x, y))

    app = QApplication(sys.argv)
    ui = Tester()
    ui.show()
    sys.exit(app.exec_())
