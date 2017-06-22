import dynamic_reconfigure.client
from PyQt5.QtCore import QObject, pyqtSlot

from views.various.HSV_entry import HSVParams

PACKAGE = 'autonomy'
import roslib;roslib.load_manifest(PACKAGE)


class HSV_Controller(QObject):
    def __init__(self, parent=None):
        super(HSV_Controller, self).__init__(parent)

    @pyqtSlot(HSVParams)
    def sendNewConfig(self, params=HSVParams):
        try:
            self.client = dynamic_reconfigure.client.Client('/marker_navigator', timeout=10)
            self.client.update_configuration({"h_low": params.h_low,
                                              "h_high": params.h_high,
                                              "s_low": params.s_low,
                                              "s_high": params.s_high,
                                              "v_low": params.v_low,
                                              "v_high": params.v_high})
        finally:
            pass

if __name__ == "__main__":
    cont = HSV_Controller()
