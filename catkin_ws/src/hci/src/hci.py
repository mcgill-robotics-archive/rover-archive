#!/usr/bin/python
import signal
from PyQt4 import QtCore, QtGui
import pyqtgraph as pg

from rover_window import *
from joystick_controller import *
from drive_publisher import *
from arm_publisher import *
from joystick_profile import JoystickProfile
from utilities import *
from ScienceController import *
from pan_tilt_controller import PanTiltController
from map_controller import MapController
import rospy
import os
import datetime

from std_msgs.msg import *
from sensor_msgs.msg import CompressedImage, Image
from omnicam.srv import ControlView
from arduino.msg import LimitSwitchClaw
from arduino.srv import *


class CentralUi(QtGui.QMainWindow):
    claw_open_on = QtCore.pyqtSignal()
    claw_open_off = QtCore.pyqtSignal()
    claw_close_on = QtCore.pyqtSignal()
    claw_close_off = QtCore.pyqtSignal()

    def __init__(self, parent=None):
        super(CentralUi, self).__init__(parent)

        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)

        self.ui.DriveMode.setChecked(True)
        self.ui.ackMoving.setChecked(False)

        self.controller = JoystickController()
        self.profile = JoystickProfile(self.controller)

        self.quality_timer = None
        self.addPointTimer = None
        self.controller_timer = None
        self.watchdog_timer = None
        self.redraw_signal = None

        self.sub = None
        self.param_list = []
        self.feed_topics = []
        self.feed_topics_hires = []

        self.modeId = 0

        # image
        self.imageMain = None
        self.imageLeft = None
        self.imageRight = None
        self.main_camera_subscriber = None

        self.init_ros()
        
        self.science = ScienceController(self.ui)
        self.drive_publisher = DriveController()
        self.map_controller = MapController(self.ui)
        self.init_connects()
        self.pan_tilt_control = PanTiltController()
        self.init_timers()
        self.get_feed_topic_params()

        self.master_name = parse_master_uri()
        self.arm_publisher = ArmPublisher()

        self.ui.camera_selector.setCurrentIndex(1)
        self.soil_checkbox_callback(False)
        self.rock_checkbox_callback(False)

        rospy.loginfo("HCI initialization completed")

    def init_ros(self):
        rospy.init_node('hci_window', anonymous=False)
        rospy.Subscriber('/limit_switch', LimitSwitchClaw, self.claw_callback, queue_size=1)
        self.main_camera_subscriber = rospy.Subscriber("/wide_angle/image_raw/compressed", CompressedImage, self.receive_pixmap_main)
        rospy.Subscriber("/left/image_raw/compressed", CompressedImage, self.receive_image_left)
        rospy.Subscriber("/right/image_raw/compressed", CompressedImage, self.receive_image_right)
        pass

    def claw_callback(self, msg):
        if msg.open:
            self.claw_open_on.emit()
        else:
            self.claw_open_off.emit()
        if msg.close:
            self.claw_close_on.emit()
        else:
            self.claw_close_off.emit()

    def init_connects(self):
        # joystick mode buttons signal connect
        QtCore.QObject.connect(self.ui.DriveMode, QtCore.SIGNAL("clicked()"), lambda index=0: self.set_controller_mode(index))
        QtCore.QObject.connect(self.ui.ArmBaseMode, QtCore.SIGNAL("clicked()"), lambda index=1: self.set_controller_mode(index))
        QtCore.QObject.connect(self.ui.ScienceMode, QtCore.SIGNAL("clicked()"), lambda index=2: self.set_controller_mode(index))
        QtCore.QObject.connect(self.ui.function4, QtCore.SIGNAL("clicked()"), lambda index=3: self.set_controller_mode(index))

        QtCore.QObject.connect(self.ui.screenshot, QtCore.SIGNAL("clicked()"), self.take_screenshot)
        QtCore.QObject.connect(self.ui.pointSteer, QtCore.SIGNAL("toggled(bool)"), self.set_point_steer)
        QtCore.QObject.connect(self.ui.ackreman, QtCore.SIGNAL("toggled(bool)"), self.set_ackreman)
        QtCore.QObject.connect(self.ui.skid, QtCore.SIGNAL("toggled(bool)"), self.set_skid)
        QtCore.QObject.connect(self.ui.translatory, QtCore.SIGNAL("toggled(bool)"), self.set_translatory)
        QtCore.QObject.connect(self.ui.read_sensor_button, QtCore.SIGNAL("clicked()"), self.read_voltage)

        QtCore.QObject.connect(self.ui.add_waypoint_dd, QtCore.SIGNAL("clicked()"), self.map_controller.add_coord_dd)
        QtCore.QObject.connect(self.ui.add_waypoint_dms, QtCore.SIGNAL("clicked()"), self.map_controller.add_coord_dms)
        QtCore.QObject.connect(self.ui.waypoint, QtCore.SIGNAL("clicked()"), self.map_controller.add_way_point)
        QtCore.QObject.connect(self.ui.clearMap, QtCore.SIGNAL("clicked()"), self.map_controller.clear_map)

        # camera feed selection signal connects
        QtCore.QObject.connect(self.ui.driveModeSelection, QtCore.SIGNAL("currentIndexChanged(int)"), self.set_motor_controller_mode)
        QtCore.QObject.connect(self.ui.camera_selector, QtCore.SIGNAL("currentIndexChanged(int)"), self.change_video_feed)
        QtCore.QObject.connect(self.ui.soil_container_checkbox, QtCore.SIGNAL("toggled(bool)"), self.soil_checkbox_callback)
        QtCore.QObject.connect(self.ui.rock_container_checkbox, QtCore.SIGNAL("toggled(bool)"), self.rock_checkbox_callback)

        # claw limit switches
        self.claw_close_on.connect(lambda lbl=self.ui.ClawCloseLimit: lbl_bg_norm(lbl))
        self.claw_open_on.connect(lambda lbl=self.ui.ClawOpenLimit: lbl_bg_norm(lbl))
        self.claw_close_off.connect(lambda lbl=self.ui.ClawCloseLimit: lbl_bg_red(lbl))
        self.claw_open_off.connect(lambda lbl=self.ui.ClawOpenLimit: lbl_bg_red(lbl))

        # augur limit switches
        self.science.limit_switch_up_on.connect(lambda lbl=self.ui.AugUpLim: lbl_bg_norm(lbl))
        self.science.limit_switch_down_on.connect(lambda lbl=self.ui.AugDnLim: lbl_bg_norm(lbl))
        self.science.limit_switch_up_off.connect(lambda lbl=self.ui.AugUpLim: lbl_bg_red(lbl))
        self.science.limit_switch_down_off.connect(lambda lbl=self.ui.AugDnLim: lbl_bg_red(lbl))

        # drill buttons
        QtCore.QObject.connect(self.ui.augur_drill_enable, QtCore.SIGNAL("clicked()"), self.toggle_drill)
        QtCore.QObject.connect(self.ui.reverse_drill_enable, QtCore.SIGNAL("pressed()"), self.science.reverse_drill)
        QtCore.QObject.connect(self.ui.reverse_drill_enable, QtCore.SIGNAL("released()"), self.science.deactivate_drill)

        # motor readys
        self.drive_publisher.fl_signal_ok.connect(lambda lbl=self.ui.fl_ok: lbl_bg_norm(lbl))
        self.drive_publisher.fr_signal_ok.connect(lambda lbl=self.ui.fr_ok: lbl_bg_norm(lbl))
        self.drive_publisher.ml_signal_ok.connect(lambda lbl=self.ui.ml_ok: lbl_bg_norm(lbl))
        self.drive_publisher.mr_signal_ok.connect(lambda lbl=self.ui.mr_ok: lbl_bg_norm(lbl))
        self.drive_publisher.bl_signal_ok.connect(lambda lbl=self.ui.bl_ok: lbl_bg_norm(lbl))
        self.drive_publisher.br_signal_ok.connect(lambda lbl=self.ui.br_ok: lbl_bg_norm(lbl))
        
        self.drive_publisher.fl_signal_bad.connect(lambda lbl=self.ui.fl_ok: lbl_bg_red(lbl))
        self.drive_publisher.fr_signal_bad.connect(lambda lbl=self.ui.fr_ok: lbl_bg_red(lbl))
        self.drive_publisher.ml_signal_bad.connect(lambda lbl=self.ui.ml_ok: lbl_bg_red(lbl))
        self.drive_publisher.mr_signal_bad.connect(lambda lbl=self.ui.mr_ok: lbl_bg_red(lbl))
        self.drive_publisher.bl_signal_bad.connect(lambda lbl=self.ui.bl_ok: lbl_bg_red(lbl))
        self.drive_publisher.br_signal_bad.connect(lambda lbl=self.ui.br_ok: lbl_bg_red(lbl))

    def init_timers(self):
        # signal quality timer
        self.quality_timer = QtCore.QTimer()
        QtCore.QObject.connect(self.quality_timer, QtCore.SIGNAL("timeout()"), self.get_signal_quality)
        self.quality_timer.start(1000)


        # self.watchdog_timer = QtCore.QTimer()
        # QtCore.QObject.connect(self.watchdog_timer, QtCore.SIGNAL("timeout()"), reset_watchdog)
        # self.watchdog_timer.start(100)

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

    def set_motor_controller_mode(self, mode_index):
        if mode_index == 0:
            self.drive_publisher.set_motor_controller_mode(MotorControllerTypeMode.SlowSpeed)
        elif mode_index == 1:
            self.drive_publisher.set_motor_controller_mode(MotorControllerTypeMode.MediumSpeed)
        elif mode_index == 2:
            self.drive_publisher.set_motor_controller_mode(MotorControllerTypeMode.HighSpeed)
        elif mode_index == 3:
            self.drive_publisher.set_motor_controller_mode(MotorControllerTypeMode.OpenLoop)

    def soil_checkbox_callback(self, open):
        if open:
            self.science.open_soil_gate()
            self.ui.soil_container_status_label.setText("Opened")
        else:
            self.science.close_soil_gate()
            self.ui.soil_container_status_label.setText("Closed")

    def rock_checkbox_callback(self, open):
        if open:
            self.science.open_rock_gate()
            self.ui.rock_container_status_label.setText("Opened")
        else:
            self.science.close_rock_gate()
            self.ui.rock_container_status_label.setText("Closed")

    def read_voltage(self):
        try:
            rospy.wait_for_service("/science/sensor_server", timeout=5)
            service = rospy.ServiceProxy("/science/sensor_server", sensor)
        except rospy.ROSException:
            rospy.logerr("Timeout, service /science/sensor_server unavailable")
            return

        response = service()
        # response = sensorResponse()
        message = "Altitude: {0} m\nPressure: {1} kPa\nAmbiant Temperature: {2} ".format(
            response.altitude,
            response.pressure,
            response.ambiant_temperature)
        message += str(chr(176))
        message += "C\nSoil Ph: {0}\nGround Temperature: {1} ".format(response.ph,
                                                                      response.ground_temperature)
        message = message + str(chr(176)) + "C\nHumidity: {0}".format(response.humidity)

        QtGui.QMessageBox.information(None, "Sensor status", message, QtGui.QMessageBox.Ok)

    def take_screenshot(self):
        topic = self.feed_topics_hires[self.ui.camera_selector.currentIndex()]
        self.sub = rospy.Subscriber(topic, Image, self.screenshot_callback, queue_size=1)
        rospy.loginfo("created screenshot subscriber " + topic)

    def screenshot_callback(self, msg):
        self.sub.unregister()
        image = QtGui.QImage(msg.data, msg.width, msg.height, QtGui.QImage.Format_RGB888)
        rospy.logerr(msg.width)
        rospy.logerr(msg.height)
        time = datetime.datetime.now().strftime("%H:%M:%S")
        topic = self.feed_topics_hires[self.ui.camera_selector.currentIndex()]
        filename = "/home/david/screenshot_" + time + ".jpg"
        save = image.save(filename)
        if save:
            rospy.logwarn("save successful " + filename)
        else:
            rospy.logwarn("fail save " + filename)

    def get_signal_quality(self):
        s = os.popen("ping -c 1 " + self.master_name)
        s.readline()
        k = s.readline()
        temp = k.split('=')
        res = temp[-1].split(' ')
        result = res[0]
        self.ui.sig_qual.setText("%s ms" % result)

    def set_point_steer(self, boolean):
        if boolean:
            self.drive_publisher.set_steering_condition(SteeringCondition.Point)

    def set_ackreman(self, boolean):
        if boolean:
            self.drive_publisher.set_steering_condition(SteeringCondition.Ackerman)

    def set_skid(self, boolean):
        if boolean:
            self.drive_publisher.set_steering_condition(SteeringCondition.Skid)

    def set_translatory(self, boolean):
        if boolean:
            self.drive_publisher.set_steering_condition(SteeringCondition.Translation)

    def read_controller(self):
        self.controller.update()
        self.profile.update_values()

        if self.profile.param_value["joystick/ackreman_moving"]:
                self.ui.ackMoving.setChecked(not self.ui.ackMoving.isChecked())

        if self.profile.param_value["joystick/drive_mode"]:
            self.set_controller_mode(0)
        elif self.profile.param_value["joystick/arm_base_mode"]:
            self.set_controller_mode(1)
        elif self.profile.param_value["joystick/camera_mode"]:
            self.set_controller_mode(3)
        elif self.profile.param_value["joystick/science_mode"]:
            self.set_controller_mode(2)

        if self.profile.param_value["joystick/point_steer"]:
            self.set_controller_mode(0)
            self.ui.pointSteer.setChecked(True)

        if self.profile.param_value["joystick/ackreman"]:
            self.set_controller_mode(0)
            self.ui.ackreman.setChecked(True)

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

        if self.modeId == 0:
            self.drive_publisher.set_enable(self.ui.ackMoving.isChecked())
            self.drive_publisher.set_speed(self.controller.a2 * 5, self.controller.a1)
            # drive mode

        elif self.modeId == 1:
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

        if self.modeId == 2:
            # Science mode.
            # Activate or not
            if self.profile.param_value["joystick/drill_on"]:
                self.ui.augur_drill_enable.setChecked(self.science.activate_drill())
                self.ui.augur_drill_status.setText("Enabled")

            if self.profile.param_value["joystick/drill_off"]:
                self.ui.augur_drill_enable.setChecked(self.science.deactivate_drill())
                self.ui.augur_drill_status.setText("Disabled")

            # todo: confirm proper axis and sign
            self.science.publish_auger_height(self.controller.a2)

        elif self.modeId == 3:
            # camera mode
            # todo: confirm proper sign
            self.pan_tilt_control.publish_pan_tilt(self.controller.a1, self.controller.a2)
            pass

        if self.profile.param_value["joystick/prev_cam"]:
            self.ui.camera_selector.setCurrentIndex((self.ui.camera_selector.currentIndex() - 1) % self.ui.camera_selector.count())
        elif self.profile.param_value["joystick/next_cam"]:
            self.ui.camera_selector.setCurrentIndex((self.ui.camera_selector.currentIndex() + 1) % self.ui.camera_selector.count())

        self.controller.clear_buttons()

    def toggle_drill(self):
        if not self.ui.augur_drill_enable.isChecked():
            self.ui.augur_drill_enable.setChecked(self.science.deactivate_drill())
            self.ui.augur_drill_status.setText("Disabled")
        else:
            self.ui.augur_drill_enable.setChecked(self.science.activate_drill())
            self.ui.augur_drill_status.setText("Enabled")

    def get_feed_topic_params(self):
        for index in xrange(0, self.ui.camera_selector.count()):
            box_text = self.ui.camera_selector.itemText(index)
            param_value = rospy.get_param(box_text, "")
            self.param_list.append(box_text)
            self.feed_topics.append(param_value)
            hires_topic = box_text+"_hires"
            param_value = rospy.get_param(hires_topic, "")
            self.feed_topics_hires.append(param_value)

    def change_video_feed(self, index):
        if index == 0:
            self.ui.rot180.setChecked(True)
            self.ui.flip_vertical.setChecked(True)
        else:
            self.ui.rot0.setChecked(True)
            self.ui.flip_vertical.setChecked(False)

        next_topic = self.feed_topics[index]

        if next_topic is not "":
            self.main_camera_subscriber.unregister()
            self.main_camera_subscriber = rospy.Subscriber(next_topic, CompressedImage, self.receive_pixmap_main)

    def set_controller_mode(self, mode_id):
        self.modeId = mode_id
        if mode_id == 0:
            self.ui.DriveMode.setChecked(True)
            self.ui.ArmBaseMode.setChecked(False)
            self.ui.ScienceMode.setChecked(False)
            self.ui.function4.setChecked(False)
        if mode_id == 1:
            self.ui.DriveMode.setChecked(False)
            self.ui.ArmBaseMode.setChecked(True)
            self.ui.ScienceMode.setChecked(False)
            self.ui.function4.setChecked(False)
        if mode_id == 2:
            self.ui.DriveMode.setChecked(False)
            self.ui.ArmBaseMode.setChecked(False)
            self.ui.ScienceMode.setChecked(True)
            self.ui.function4.setChecked(False)
        if mode_id == 3:
            self.ui.DriveMode.setChecked(False)
            self.ui.ArmBaseMode.setChecked(False)
            self.ui.ScienceMode.setChecked(False)
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

                if self.ui.flip_vertical.isChecked():
                    pixmap_main = pixmap_main.transformed(QtGui.QTransform().scale(-1, 1))  # mirror on the y axis

                pixmap_main = pixmap_main.scaled(QtCore.QSize(pixmap_main.width() * 2, pixmap_main.height() * 2), 0)

                if self.ui.rot0.isChecked():
                    self.ui.camera1.setPixmap(pixmap_main)
                elif self.ui.rot90.isChecked():
                    rotated = pixmap_main.transformed(QtGui.QMatrix().rotate(90), QtCore.Qt.SmoothTransformation)
                    self.ui.camera1.setPixmap(rotated)
                elif self.ui.rot180.isChecked():
                    rotated = pixmap_main.transformed(QtGui.QMatrix().rotate(180), QtCore.Qt.SmoothTransformation)
                    self.ui.camera1.setPixmap(rotated)
                elif self.ui.rot270.isChecked():
                    rotated = pixmap_main.transformed(QtGui.QMatrix().rotate(270), QtCore.Qt.SmoothTransformation)
                    self.ui.camera1.setPixmap(rotated)

            finally:
                pass
        else:
            self.ui.camera1.setText("no video feed")

        if self.imageLeft is not None:
            try:
                qimageTop = QtGui.QImage.fromData(self.imageLeft.data)
                imageTop = QtGui.QPixmap.fromImage(qimageTop)
                rotated = imageTop.transformed(QtGui.QMatrix().rotate(-90), QtCore.Qt.SmoothTransformation)
                rotated = rotated.scaled(QtCore.QSize(rotated.width() * 2, rotated.height() * 2), 0)
                # left_painter = QtGui.QPainter(rotated)
                # left_painter.drawPixmap(0, 0, self.overlay_pixmap)
                self.ui.camera2.setPixmap(rotated)
                # left_painter.end()
            finally:
                pass

        else:
            self.ui.camera2.setText("no video feed")

        if self.imageRight is not None:
            try:
                qimageBottom = QtGui.QImage.fromData(self.imageRight.data)
                imageBottom = QtGui.QPixmap.fromImage(qimageBottom)
                rotated = imageBottom.transformed(QtGui.QMatrix().rotate(90), QtCore.Qt.SmoothTransformation)
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
