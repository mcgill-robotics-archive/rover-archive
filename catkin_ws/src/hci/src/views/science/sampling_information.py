import sys
from PyQt5.QtCore import Qt, pyqtSlot

from PyQt5.QtGui import QFont
from PyQt5.QtWidgets import QApplication
from PyQt5.QtWidgets import QFrame
from PyQt5.QtWidgets import QGridLayout
from PyQt5.QtWidgets import QHBoxLayout
from PyQt5.QtWidgets import QLabel
from PyQt5.QtWidgets import QVBoxLayout
from PyQt5.QtWidgets import QWidget


class SamplingInformation(QWidget):
    def __init__(self, parent=None):
        super(SamplingInformation, self).__init__(parent)

        title_lbl = QLabel("Sampling Status", self)
        car_lbl = QLabel("Carriage Speed:", self)
        dri_lbl = QLabel("Drill Speed:", self)
        pro_lbl = QLabel("Probe Speed:", self)
        inc_dr_lbl = QLabel("Drill Position:", self)
        inc_cr_lbl = QLabel("Carriage Position:", self)

        self.car_ind = QLabel(self)
        self.dri_ind = QLabel(self)
        self.pro_ind = QLabel(self)
        self.inc_dr_ind = QLabel(self)
        self.inc_cr_ind = QLabel(self)

        title_font = QFont()
        title_font.setPointSize(15)
        title_font.setBold(True)
        title_font.setWeight(75)

        title_lbl.setFont(title_font)
        v1 = QVBoxLayout()
        g1 = QGridLayout()

        g1.addWidget(car_lbl, 0, 0)
        g1.addWidget(dri_lbl, 1, 0)
        g1.addWidget(pro_lbl, 2, 0)
        g1.addWidget(inc_cr_lbl, 0, 2)
        g1.addWidget(inc_dr_lbl, 1, 2)

        g1.addWidget(self.car_ind, 0, 1)
        g1.addWidget(self.dri_ind, 1, 1)
        g1.addWidget(self.pro_ind, 2, 1)
        g1.addWidget(self.inc_cr_ind, 0, 3)
        g1.addWidget(self.inc_dr_ind, 1, 3)

        v1.addWidget(title_lbl)
        v1.addItem(g1)

        line_2 = QFrame(self)
        line_2.setFrameShape(QFrame.HLine)
        line_2.setFrameShadow(QFrame.Sunken)
        v1.addWidget(line_2)


        line_1 = QFrame(self)
        line_1.setFrameShape(QFrame.HLine)
        line_1.setFrameShadow(QFrame.Sunken)

        title_lbl = QLabel("Sensor Values", self)
        title_lbl.setFont(title_font)
        v1.addWidget(title_lbl)

        wind_lbl = QLabel("Wind Speed:", self)
        humi_lbl = QLabel("Humidity:", self)
        temp_lbl = QLabel("Temperature:", self)
        self.wind_ind = QLabel(self)
        self.humi_ind = QLabel(self)
        self.temp_ind = QLabel(self)

        h1 = QHBoxLayout()
        h1.addWidget(wind_lbl)
        h1.addWidget(self.wind_ind)
        h1.addWidget(humi_lbl)
        h1.addWidget(self.humi_ind)
        h1.addWidget(temp_lbl)
        h1.addWidget(self.temp_ind)
        v1.addItem(h1)
        v1.addWidget(line_1)
        self.setLayout(v1)

    @pyqtSlot(int)
    def update_drill_position(self, position):
        self.inc_dr_ind.setText(str(position))

    @pyqtSlot(int)
    def update_carriage_position(self, position):
        self.inc_cr_ind.setText(str(position))

    @pyqtSlot(int)
    def update_drill_speed(self, position):
        self.dri_ind.setText(str(position))

    @pyqtSlot(int)
    def update_carriage_speed(self, position):
        self.car_ind.setText(str(position))

    @pyqtSlot(int)
    def update_probe_speed(self, position):
        self.pro_ind.setText(str(position))

    @pyqtSlot(int)
    def update_humidity(self, position):
        self.humi_ind.setText(str(position))

    @pyqtSlot(int)
    def update_wind(self, position):
        self.wind_ind.setText(str(position))

    @pyqtSlot(int)
    def update_temperature(self, position):
        self.temp_ind.setText(str(position))

if __name__ == "__main__":
    app = QApplication(sys.argv)
    ui = SamplingInformation()
    ui.show()
    ui.update_drill_position(5)
    ui.update_drill_speed(2)
    sys.exit(app.exec_())