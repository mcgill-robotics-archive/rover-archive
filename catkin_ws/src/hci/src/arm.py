#!/usr/bin/python
import signal
from PyQt4 import QtCore, QtGui
import pyqtgraph as pg

from arm_gui import *
from joystick_controller import *
from arm_publisher import *
from joystick_profile import JoystickProfile
from utilities import *

import rospy
import Queue
import os
import math
import datetime

from std_msgs.msg import *
from rover_common.msg import MotorStatus
from rover_camera.srv import ChangeFeed
from rover_common.srv import GetVoltageRead
from sensor_msgs.msg import CompressedImage, Image
from omnicam.srv import ControlView


class CentralUi(QtGui.QMainWindow):

    fl_signal_ok = QtCore.pyqtSignal()
    fr_signal_ok = QtCore.pyqtSignal()
    ml_signal_ok = QtCore.pyqtSignal()
    mr_signal_ok = QtCore.pyqtSignal()
    bl_signal_ok = QtCore.pyqtSignal()
    br_signal_ok = QtCore.pyqtSignal()

    fl_signal_bad = QtCore.pyqtSignal()
    fr_signal_bad = QtCore.pyqtSignal()
    ml_signal_bad = QtCore.pyqtSignal()
    mr_signal_bad = QtCore.pyqtSignal()
    bl_signal_bad = QtCore.pyqtSignal()
    br_signal_bad = QtCore.pyqtSignal()

    def __init__(self, parent=None):
        super(CentralUi, self).__init__(parent)

        self.points_counter = 0
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)

        self.ui.ArmBaseMode.setChecked(True)
        self.ui.ackMoving.setChecked(False)

        self.controller = JoystickController()
        self.profile = JoystickProfile(self.controller)

        self.quality_timer = None
        self.controller_timer = None
        self.watchdog_timer = None
        self.redraw_signal = None

        self.sub = None
        self.param_list = []
        self.feed_topics = []
        self.feed_topics_hires = []

        self.modeId = 1

        # image
        self.imageMain = None
        self.imageLeft = None
        self.imageRight = None
        self.image4 = None
        self.main_camera_subscriber = None

        self.init_ros()
        self.init_connects()
        self.init_timers()

        self.master_name = parse_master_uri()
        self.arm_publisher = ArmPublisher()

        rospy.loginfo("HCI initialization completed")

    def init_ros(self):
        rospy.init_node('hci_window', anonymous=False)
        rospy.Subscriber('/motor_status', MotorStatus, self.motor_status, queue_size=10)
        self.main_camera_subscriber = rospy.Subscriber("/wide_angle/image_color/compressed", CompressedImage, self.receive_pixmap_main)
        rospy.Subscriber("/side/image_color/compressed", CompressedImage, self.receive_image_side)
        rospy.Subscriber("/omnicam/image_color/compressed", CompressedImage, self.receive_image_left)
        rospy.Subscriber("/camera_arm/image_color/compressed", CompressedImage, self.receive_image_right)
        pass

    def motor_status(self, msg):
        if msg.fl:
            self.fl_signal_ok.emit()
        else:
            self.fl_signal_bad.emit()

        if msg.fr:
            self.fr_signal_ok.emit()
        else:
            self.fr_signal_bad.emit()

        if msg.ml:
            self.ml_signal_ok.emit()
        else:
            self.ml_signal_bad.emit()

        if msg.mr:
            self.mr_signal_ok.emit()
        else:
            self.mr_signal_bad.emit()

        if msg.bl:
            self.bl_signal_ok.emit()
        else:
            self.bl_signal_bad.emit()

        if msg.br:
            self.br_signal_ok.emit()
        else:
            self.br_signal_bad.emit()

    def init_connects(self):
        # joystick mode buttons signal connect
        QtCore.QObject.connect(self.ui.ArmBaseMode, QtCore.SIGNAL("clicked()"),
                               lambda index=1: self.set_controller_mode(index))
        QtCore.QObject.connect(self.ui.function4, QtCore.SIGNAL("clicked()"),
                               lambda index=3: self.set_controller_mode(index))

        # motor readys
        self.fl_signal_ok.connect(lambda lbl=self.ui.fl_ok: lbl_bg_norm(lbl))
        self.fr_signal_ok.connect(lambda lbl=self.ui.fr_ok: lbl_bg_norm(lbl))
        self.ml_signal_ok.connect(lambda lbl=self.ui.ml_ok: lbl_bg_norm(lbl))
        self.mr_signal_ok.connect(lambda lbl=self.ui.mr_ok: lbl_bg_norm(lbl))
        self.bl_signal_ok.connect(lambda lbl=self.ui.bl_ok: lbl_bg_norm(lbl))
        self.br_signal_ok.connect(lambda lbl=self.ui.br_ok: lbl_bg_norm(lbl))

        self.fl_signal_bad.connect(lambda lbl=self.ui.fl_ok: lbl_bg_red(lbl))
        self.fr_signal_bad.connect(lambda lbl=self.ui.fr_ok: lbl_bg_red(lbl))
        self.ml_signal_bad.connect(lambda lbl=self.ui.ml_ok: lbl_bg_red(lbl))
        self.mr_signal_bad.connect(lambda lbl=self.ui.mr_ok: lbl_bg_red(lbl))
        self.bl_signal_bad.connect(lambda lbl=self.ui.bl_ok: lbl_bg_red(lbl))
        self.br_signal_bad.connect(lambda lbl=self.ui.br_ok: lbl_bg_red(lbl))

    def init_timers(self):
        # signal quality timer
        self.quality_timer = QtCore.QTimer()
        QtCore.QObject.connect(self.quality_timer, QtCore.SIGNAL("timeout()"), self.get_signal_quality)
        self.quality_timer.start(1000)

        # controller timer connect
        self.controller_timer = QtCore.QTimer()
        QtCore.QObject.connect(self.controller_timer, QtCore.SIGNAL("timeout()"), self.read_controller)

        self.redraw_signal = QtCore.QTimer(self)
        QtCore.QObject.connect(self.redraw_signal, QtCore.SIGNAL("timeout()"), self.repaint_image)
        self.redraw_signal.start(100)

        if self.controller.controller is not None:
            self.controller_timer.start(100)
            rospy.loginfo("Started controller timer")
        else:
            rospy.logwarn("Missing controller, timer aborted")

    def get_signal_quality(self):
        s = os.popen("ping -c 1 " + self.master_name)
        s.readline()
        k = s.readline()
        temp = k.split('=')
        res = temp[-1].split(' ')
        result = res[0]
        self.ui.sig_qual.setText("%s ms" % result)

    def read_controller(self):
        self.controller.update()
        self.profile.update_values()

        if self.profile.param_value["joystick/ackreman_moving"]:
                self.ui.ackMoving.setChecked(not self.ui.ackMoving.isChecked())

        if self.profile.param_value["joystick/arm_base_mode"]:
            self.set_controller_mode(1)
        elif self.profile.param_value["joystick/camera_mode"]:
            self.set_controller_mode(3)

        if self.profile.param_value["logitech/base"]:
            self.set_controller_mode(1)
            self.ui.base.setChecked(True)

        if self.profile.param_value["logitech/diff1"]:
            self.set_controller_mode(1)
            self.ui.diff1.setChecked(True)

        if self.profile.param_value["logitech/diff2"]:
            self.set_controller_mode(1)
            self.ui.diff2.setChecked(True)

        if self.profile.param_value["logitech/end"]:
            self.set_controller_mode(1)
            self.ui.end_eff.setChecked(True)

        self.controller.clear_buttons()
        self.publish_controls()

    def publish_controls(self):
        if self.modeId == 1:
            # arm base mode
            if self.ui.ackMoving.isChecked():
                constant = (self.controller.a4 + 1) * 100
                rospy.logdebug("Scalar constant : {0}".format(constant))
                if self.ui.base.isChecked():
                    self.arm_publisher.publish_base(self.controller.a2 * constant, -self.controller.a1 * constant)
                elif self.ui.diff1.isChecked():
                    self.arm_publisher.publish_diff_1(self.controller.a2 * constant, self.controller.a1 * constant)
                elif self.ui.diff2.isChecked():
                    self.arm_publisher.publish_diff_2(self.controller.a2 * constant, self.controller.a1 * constant)
                elif self.ui.end_eff.isChecked():
                    self.arm_publisher.publish_end_effector(self.controller.a2 * constant)

            else:
                self.arm_publisher.publish_joint_vels(0, 0, 0, 0, 0, 0, 0)

        elif self.modeId == 3:
            # camera mode
            if (self.controller.a2 != 0) or (self.controller.a3 != 0) or (self.controller.a1 != 0):
                try:
                    rospy.wait_for_service("crop_control", timeout=1)
                    service = rospy.ServiceProxy("crop_control", ControlView)
                except rospy.ROSException:
                    rospy.logerr("Timeout trying to find service /crop_control")
                    return

                response = service(-10 * self.controller.a1, -10 * self.controller.a2, 10 * self.controller.a3)
                if not response:
                    rospy.logerr("Failed to adjust omnicam image.")

    def set_controller_mode(self, mode_id):
        self.modeId = mode_id
        if mode_id == 1:
            self.ui.ArmBaseMode.setChecked(True)
            self.ui.function4.setChecked(False)
        if mode_id == 3:
            self.ui.ArmBaseMode.setChecked(False)
            self.ui.function4.setChecked(True)

    def receive_pixmap_main(self, data):
        try:
            self.imageMain = data
        finally:
            pass

    def receive_image_left(self, data):
        try:
            self.imageLeft = data
        finally:
            pass

    def receive_image_side(self, data):
        try:
            self.image4 = data
        finally:
            pass

    def receive_image_right(self, data):
        try:
            self.imageRight = data
        finally:
            pass

    def repaint_image(self):
        if self.imageMain is not None:
            try:
                qimageMain = QtGui.QImage.fromData(self.imageMain.data)
                pixmap_main = QtGui.QPixmap.fromImage(qimageMain)

                # if self.ui.flip_vertical.isChecked():
                #     pixmap_main = pixmap_main.transformed(QtGui.QTransform().scale(-1, 1))  # mirror on the y axis

                pixmap_main = pixmap_main.scaled(QtCore.QSize(pixmap_main.width() * 2, pixmap_main.height() * 2), 0)

                # if self.ui.rot0.isChecked():
                #self.ui.camera1.setPixmap(pixmap_main)
                # elif self.ui.rot90.isChecked():
                #     rotated = pixmap_main.transformed(QtGui.QMatrix().rotate(90), QtCore.Qt.SmoothTransformation)
                #     self.ui.camera1.setPixmap(rotated)
                # elif self.ui.rot180.isChecked():
                rotated = pixmap_main#.transformed(QtGui.QMatrix().rotate(180), QtCore.Qt.SmoothTransformation)
                self.ui.camera1.setPixmap(rotated)
                # elif self.ui.rot270.isChecked():
                #     rotated = pixmap_main.transformed(QtGui.QMatrix().rotate(270), QtCore.Qt.SmoothTransformation)
                #     self.ui.camera1.setPixmap(rotated)

            finally:
                pass
        else:
            self.ui.camera1.setText("no video feed")

        if self.imageLeft is not None:
            try:
                qimageTop = QtGui.QImage.fromData(self.imageLeft.data)
                imageTop = QtGui.QPixmap.fromImage(qimageTop)
                rotated = imageTop#.transformed(QtGui.QMatrix().rotate(-90), QtCore.Qt.SmoothTransformation)
                rotated = rotated.scaled(QtCore.QSize(rotated.width() * 2, rotated.height() * 2), 0)
                # left_painter = QtGui.QPainter(rotated)
                # left_painter.drawPixmap(0, 0, self.overlay_pixmap)
                self.ui.camera2.setPixmap(rotated)
                # left_painter.end()
            finally:
                pass

        else:
            self.ui.camera2.setText("no video feed")

        if self.image4 is not None:
            try:
                qimageBottom2 = QtGui.QImage.fromData(self.image4.data)
                imageBottom2 = QtGui.QPixmap.fromImage(qimageBottom2)
                rotated = imageBottom2.transformed(QtGui.QMatrix().rotate(180), QtCore.Qt.SmoothTransformation)
                rotated = rotated.scaled(QtCore.QSize(rotated.width() * 2, rotated.height() * 2), 0)
            finally:
                pass
            self.ui.camera4.setPixmap(rotated)
        else:
            # self.ui.camera3.setPixmap(self.overlay_pixmap)
            self.ui.camera4.setText("no video feed")

            if self.imageRight is not None:
                try:
                    qimageBottom = QtGui.QImage.fromData(self.imageRight.data)
                    rotated = QtGui.QPixmap.fromImage(qimageBottom)
                    # rotated = imageBottom  # .transformed(QtGui.QMatrix().rotate(90), QtCore.Qt.SmoothTransformation)
                    rotated = rotated.scaled(QtCore.QSize(rotated.width() * 2, rotated.height() * 2), 0)
                finally:
                    pass
                self.ui.camera3.setPixmap(rotated)
            else:
                # self.ui.camera3.setPixmap(self.overlay_pixmap)
                self.ui.camera3.setText("no video feed")


def sigint_handler(*args):
    """Handler for the SIGINT signal."""
    sys.stderr.write('\r')
    # if QtGui.QMessageBox.question(None, '', "Are you sure you want to quit?",
    #                              QtGui.QMessageBox.Yes | QtGui.QMessageBox.No,
    #                              QtGui.QMessageBox.Yes) == QtGui.QMessageBox.Yes:
    rospy.loginfo("[Front-end] EXITING")
    QtGui.QApplication.quit()


if __name__ == "__main__":
    signal.signal(signal.SIGINT, sigint_handler)
    app = QtGui.QApplication(sys.argv)

    AppWindow = CentralUi()
    AppWindow.show()
    sys.exit(app.exec_())
