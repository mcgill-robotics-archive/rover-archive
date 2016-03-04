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

        # map place holders
        self.tempPose = Queue.Queue()
        self.new_x = []
        self.new_y = []
        self.w1 = None
        self.s1 = None

        self.x_waypoints = []
        self.y_waypoints = []

        self.first_point = False
        self.dx = 0
        self.dy = 0
        # list for set of points in mini-map
        self.map_point_list = []

        self.init_ros()
        self.init_connects()
        self.init_timers()
        self.setup_minimap()
        self.get_feed_topic_params()

        self.master_name = parse_master_uri()
        self.drive_publisher = DrivePublisher()
        self.arm_publisher = ArmPublisher()

        path = os.environ.get('ROBOTIC_PATH') + "/rover/catkin_ws/src/hci/src/grid_vertical.png"
        self.overlay_pixmap = QtGui.QPixmap(path)
        if self.overlay_pixmap.isNull():
            rospy.logerr("Pixmap empty")
            rospy.logerr(path)

        rospy.loginfo("HCI initialization completed")

    def init_ros(self):
        rospy.init_node('hci_window', anonymous=False)
        # rospy.Subscriber('ahrs_status', AhrsStatusMessage, self.handle_pose, queue_size=10)
        rospy.Subscriber('/motor_status', MotorStatus, self.motor_status, queue_size=10)
        self.main_camera_subscriber = rospy.Subscriber("/econ", CompressedImage, self.receive_pixmap_main)
        rospy.Subscriber("/left/image_mono/compressed", CompressedImage, self.receive_image_left)
        rospy.Subscriber("/right/image_mono/compressed", CompressedImage, self.receive_image_right)
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
        QtCore.QObject.connect(self.ui.DriveMode, QtCore.SIGNAL("clicked()"),
                               lambda index=0: self.set_controller_mode(index))
        QtCore.QObject.connect(self.ui.ArmBaseMode, QtCore.SIGNAL("clicked()"),
                               lambda index=1: self.set_controller_mode(index))
        # QtCore.QObject.connect(self.ui.EndEffectorMode, QtCore.SIGNAL("clicked()"),
        #                        lambda index=2: self.set_controller_mode(index))
        QtCore.QObject.connect(self.ui.function4, QtCore.SIGNAL("clicked()"),
                               lambda index=3: self.set_controller_mode(index))

        QtCore.QObject.connect(self.ui.screenshot, QtCore.SIGNAL("clicked()"), self.take_screenshot)
        QtCore.QObject.connect(self.ui.pointSteer, QtCore.SIGNAL("toggled(bool)"), self.set_point_steer)
        QtCore.QObject.connect(self.ui.ackreman, QtCore.SIGNAL("toggled(bool)"), self.set_ackreman)
        QtCore.QObject.connect(self.ui.skid, QtCore.SIGNAL("toggled(bool)"), self.set_skid)
        QtCore.QObject.connect(self.ui.translatory, QtCore.SIGNAL("toggled(bool)"), self.set_translatory)
        QtCore.QObject.connect(self.ui.add_waypoint_dd, QtCore.SIGNAL("clicked()"), self.add_coord_dd)
        QtCore.QObject.connect(self.ui.add_waypoint_dms, QtCore.SIGNAL("clicked()"), self.add_coord_dms)
        QtCore.QObject.connect(self.ui.read_voltage_button, QtCore.SIGNAL("clicked()"), self.read_voltage)

        # camera feed selection signal connects
        QtCore.QObject.connect(self.ui.waypoint, QtCore.SIGNAL("clicked()"), self.add_way_point)
        QtCore.QObject.connect(self.ui.clearMap, QtCore.SIGNAL("clicked()"), self.clear_map)
        QtCore.QObject.connect(self.ui.driveModeSelection, QtCore.SIGNAL("currentIndexChanged(int)"),
                               self.set_motor_controller_mode)
        QtCore.QObject.connect(self.ui.camera_selector, QtCore.SIGNAL("currentIndexChanged(int)"),
                               self.change_video_feed)

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

        # add point to map
        self.addPointTimer = QtCore.QTimer()
        QtCore.QObject.connect(self.addPointTimer, QtCore.SIGNAL("timeout()"), self.add_point_timeout)
        self.addPointTimer.start(100)

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

    def read_voltage(self):
        try:
            rospy.wait_for_service("arm/get_voltage", timeout=2)
            service = rospy.ServiceProxy("arm/get_voltage", GetVoltageRead)
        except rospy.ROSException:
            rospy.logerr("Timeout, service get_voltage unavailable")
            return

        response = service()
        self.ui.input_voltage_label.setText(str(response.Voltage))

    def take_screenshot(self):
        topic = self.feed_topics_hires[self.ui.camera_selector.currentIndex()]
        self.sub = rospy.Subscriber(topic, Image, self.screenshot_callback, queue_size=1)
        rospy.loginfo("created screenshot subscriber " + topic)

    def screenshot_callback(self, msg):
        rospy.loginfo("shot callback")
        self.sub.unregister()
        image = QtGui.QImage(msg.data, msg.width, msg.height, QtGui.QImage.Format_RGB888)
        time = datetime.datetime.now().strftime("%H:%M:%S")
        topic = self.feed_topics_hires[self.ui.camera_selector.currentIndex()]
        filename = "screenshot_" + time + ".jpg"
        save = image.save(filename)
        if save:
            rospy.logwarn("save successful " + filename)
        else:
            rospy.logwarn("fail save " + filename)

    def add_point_set_to_mini_map(self):
        new_set = pg.ScatterPlotItem(size=10, pen=pg.mkPen('w'), pxMode=True)  # create new point set
        self.map_point_list.append(new_set)  # add point set to member list
        self.w1.addItem(self.map_point_list[-1])  # add point set to graph window
        rospy.loginfo("Added new scatter plot item")

    def setup_minimap(self):

        self.w1 = self.ui.graphicsView.addViewBox()
        self.w1.setAspectLocked(False)
        self.w1.enableAutoRange('xy', True)
        self.ui.graphicsView.nextRow()

        self.x_waypoints.append(0)
        self.y_waypoints.append(0)

        self.add_point_set_to_mini_map()
        self.map_point_list[-1].addPoints(self.x_waypoints, self.y_waypoints, size=10, symbol='t', brush='b')

    def handle_pose(self, data):
        if data.gpsLongitude is not 0:
            if not self.first_point:
                self.first_point = True
                self.dx = data.gpsLongitude
                self.dy = data.gpsLatitude

            # add (x,y) to tempPose queue
            self.tempPose.put(data)

    def add_point_timeout(self):
        while not self.tempPose.empty():
            pose = self.tempPose.get()

            self.new_x = [(pose.gpsLongitude - self.dx)]
            self.new_y = [(pose.gpsLatitude - self.dy)]
            self.ui.latActual.setText(format_dms(pose.gpsLatitude))
            self.ui.lonActual.setText(format_dms(pose.gpsLongitude))

            self.ui.pitchLBL.setText(format_euler_angle(pose.pitch))
            self.ui.rollLBL.setText(format_euler_angle(pose.roll))
            self.ui.yawLBL.setText(format_euler_angle(pose.yaw))

            self.points_counter += 1
            if self.points_counter % 1000 == 0:
                self.add_point_set_to_mini_map()

            self.map_point_list[-1].addPoints(self.new_x, self.new_y, size=3, symbol='o', brush='w')
            if self.ui.zoomGraph.isChecked():
                self.w1.autoRange()

    def add_way_point(self):
        self.x_waypoints.append(self.new_x[0])
        self.y_waypoints.append(self.new_y[0])
        self.map_point_list[-1].addPoints([self.new_x[0]], [self.new_y[0]], size=10, symbol='t', brush='b')
        if self.ui.zoomGraph.isChecked():
            self.w1.autoRange()

    def add_coord_dms(self):
        longitude = self.ui.lon_deg.value() + self.ui.lon_min.value() / 60.0 + self.ui.lon_sec.value() / 3600.0
        latitude = self.ui.lat_deg.value() + self.ui.lat_min.value() / 60.0 + self.ui.lat_sec.value() / 3600.0

        if self.ui.lat_sign.currentIndex() == 1:
            latitude = - latitude

        if self.ui.lon_sign.currentIndex() == 1:
            longitude = - longitude

        x = longitude - self.dx
        y = latitude - self.dy
        self.x_waypoints.append(x)
        self.y_waypoints.append(y)
        self.map_point_list[-1].addPoints([x], [y], size=10, symbol='t', brush='r')
        if self.ui.zoomGraph.isChecked():
            self.w1.autoRange()

    def add_coord_dd(self):
        x = self.ui.x.value() - self.dx
        y = self.ui.y.value() - self.dy
        self.x_waypoints.append(x)
        self.y_waypoints.append(y)
        self.map_point_list[-1].addPoints([x], [y], size=10, symbol='t', brush='r')
        if self.ui.zoomGraph.isChecked():
            self.w1.autoRange()

    def clear_map(self):
        self.first_point = False
        self.dx = 0
        self.dy = 0

        for item in self.map_point_list:
            self.w1.removeItem(item)
            self.map_point_list.remove(item)

        self.add_point_set_to_mini_map()

        self.map_point_list[-1].setData([], [], size=10, symbol='o', brush='r')
        self.map_point_list[-1].addPoints(self.x_waypoints, self.y_waypoints, size=10, symbol='t', brush='b')

    def get_signal_quality(self):
        # TODO: make the target dynamic using ros_master_uri
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

        if self.profile.param_value["/joystick/ackreman_moving"]:
                self.ui.ackMoving.setChecked(not self.ui.ackMoving.isChecked())

        if self.profile.param_value["/joystick/drive_mode"]:
            self.set_controller_mode(0)
        elif self.profile.param_value["/joystick/arm_base_mode"]:
            self.set_controller_mode(1)
        elif self.profile.param_value["/joystick/camera_mode"]:
            self.set_controller_mode(3)

        if self.profile.param_value["/joystick/point_steer"]:
            self.set_controller_mode(0)
            self.ui.pointSteer.setChecked(True)

        if self.profile.param_value["/joystick/ackreman"]:
            self.set_controller_mode(0)
            self.ui.ackreman.setChecked(True)

        if self.modeId == 0:
            if self.profile.param_value["/joystick/toggle_point_steer"]:
                if self.ui.ackreman.isChecked():
                    self.ui.pointSteer.setChecked(True)
                else:
                    self.ui.ackreman.setChecked(True)
            pass

        elif self.modeId == 1:
            if self.ui.pitch1.isChecked():
                self.arm_publisher.publish_base_pitch(self.controller.a2 * 100)
            elif self.ui.diff1.isChecked():
                self.arm_publisher.publish_diff_1(self.controller.a2 * 100, self.controller.a1 * 100)
            elif self.ui.diff2.isChecked():
                self.arm_publisher.publish_diff_2(self.controller.a2 * 100, self.controller.a1 * 100)
            elif self.ui.end_eff.isChecked():
                self.arm_publisher.publish_end_effector(self.controller.a2 * 100)

        elif self.modeId == 3:
            # currently in camera control
            if self.profile.param_value["joystick/prev_cam"]:
                self.ui.camera_selector.setCurrentIndex((self.ui.camera_selector.currentIndex() - 1) % self.ui.camera_selector.count())
            elif self.profile.param_value["joystick/next_cam"]:
                self.ui.camera_selector.setCurrentIndex((self.ui.camera_selector.currentIndex() + 1) % self.ui.camera_selector.count())


        self.controller.clear_buttons()
        self.publish_controls()

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
            self.ui.flip_vertical.setChecked(True)
        else:
            self.ui.flip_vertical.setChecked(False)

        next_topic = self.feed_topics[index]
        try:
            rospy.wait_for_service("/changeFeed", timeout=2)
            service = rospy.ServiceProxy("/changeFeed", ChangeFeed)
        except rospy.ROSException:
            rospy.logerr("Timeout trying to find service /changeFeed")
            return

        if next_topic is not "":
            self.main_camera_subscriber.unregister()
            self.main_camera_subscriber = rospy.Subscriber(next_topic, CompressedImage, self.receive_pixmap_main)

        param = self.param_list[index]
        response = service("/" + str(param))
        print response

    def publish_controls(self):
        if self.modeId == 0:
            self.drive_publisher.set_enable(self.ui.ackMoving.isChecked())
            self.drive_publisher.set_speed(-self.controller.a2, -self.controller.a1)
            # drive mode
            pass
        elif self.modeId == 1:
            # arm base mode
            pass
        elif self.modeId == 2:
            # end effector mode
            pass
        elif self.modeId == 3:
            # camera mode
            if (self.controller.a2 != 0) or (self.controller.a3 != 0) or (self.controller.a1 != 0):
                try:
                    rospy.wait_for_service("/omnicam/crop_control", timeout=2)
                    service = rospy.ServiceProxy("/omnicam/crop_control", ControlView)
                except rospy.ROSException:
                    rospy.logerr("Timeout trying to find service /omnicam/crop_control")
                    return

                response = service(-10 * self.controller.a1, -10 * self.controller.a2, 10 * self.controller.a3)
                if not response:
                    rospy.logerr("Failed to adjust omnicam image.")
            pass

    def set_controller_mode(self, mode_id):
        self.modeId = mode_id
        if mode_id == 0:
            self.ui.DriveMode.setChecked(True)
            self.ui.ArmBaseMode.setChecked(False)
            self.ui.EndEffectorMode.setChecked(False)
            self.ui.function4.setChecked(False)
        if mode_id == 1:
            self.ui.DriveMode.setChecked(False)
            self.ui.ArmBaseMode.setChecked(True)
            self.ui.EndEffectorMode.setChecked(False)
            self.ui.function4.setChecked(False)
        if mode_id == 2:
            self.ui.DriveMode.setChecked(False)
            self.ui.ArmBaseMode.setChecked(False)
            self.ui.EndEffectorMode.setChecked(True)
            self.ui.function4.setChecked(False)
        if mode_id == 3:
            self.ui.DriveMode.setChecked(False)
            self.ui.ArmBaseMode.setChecked(False)
            self.ui.EndEffectorMode.setChecked(False)
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
                left_painter = QtGui.QPainter(rotated)
                left_painter.drawPixmap(0, 0, self.overlay_pixmap)
                self.ui.camera2.setPixmap(rotated)
                left_painter.end()
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
            self.ui.camera3.setPixmap(self.overlay_pixmap)
            # self.ui.camera3.setText("no video feed")


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
