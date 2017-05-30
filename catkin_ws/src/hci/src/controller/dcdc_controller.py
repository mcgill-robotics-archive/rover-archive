import rospy
from PyQt5.QtCore import QObject
from PyQt5.QtCore import pyqtSignal
from std_msgs.msg import Float64


class DCDC_Controller(QObject):
    updateIv = pyqtSignal(float)
    updateIc = pyqtSignal(float)
    updateTemp = pyqtSignal(float)
    updateOv = pyqtSignal(float)
    updateOc = pyqtSignal(float)
    updateOp = pyqtSignal(float)

    def __init__(self, parent=None):
        super(DCDC_Controller, self).__init__(parent)

        iv_sub = rospy.Subscriber("/dcdc_nuc/input_voltage", Float64, self.new_iv)
        ic_sub = rospy.Subscriber("/dcdc_nuc/input_current", Float64, self.new_ic)
        ov_sub = rospy.Subscriber("/dcdc_nuc/output_voltage", Float64, self.new_ov)
        oc_sub = rospy.Subscriber("/dcdc_nuc/output_current", Float64, self.new_oc)
        op_sub = rospy.Subscriber("/dcdc_nuc/output_power", Float64, self.new_op)
        te_sub = rospy.Subscriber("/dcdc_nuc/temperature", Float64, self.new_temp)

    def new_iv(self, value):
        self.updateIv.emit(value.data)

    def new_ic(self, value):
        self.updateIc.emit(value.data)

    def new_ov(self, value):
        self.updateOv.emit(value.data)

    def new_oc(self, value):
        self.updateOc.emit(value.data)

    def new_op(self, value):
        self.updateOp.emit(value.data)

    def new_temp(self, value):
        self.updateTemp.emit(value.data)
