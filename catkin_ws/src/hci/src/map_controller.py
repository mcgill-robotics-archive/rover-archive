from PyQt4 import QtCore
import pyqtgraph as pg
from ahrs.msg import AhrsStdMsg
from utilities import *
import tf.transformations
import Queue


class MapController(QtCore.QObject):
    def __init__(self, view):
        QtCore.QObject.__init__(self)

        self.view = view
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

        self.setup_minimap()

        rospy.Subscriber("/ahrs/ahrs_status", AhrsStdMsg, self.handle_pose, queue_size=5)

        # add point to map
        self.addPointTimer = QtCore.QTimer()
        QtCore.QObject.connect(self.addPointTimer, QtCore.SIGNAL("timeout()"), self.add_point_timeout)
        self.addPointTimer.start(100)

    def add_point_set_to_mini_map(self):
        new_set = pg.ScatterPlotItem(size=10, pen=pg.mkPen('w'), pxMode=True)  # create new point set
        self.map_point_list.append(new_set)  # add point set to member list
        self.w1.addItem(self.map_point_list[-1])  # add point set to graph window
        rospy.loginfo("Added new scatter plot item")

    def setup_minimap(self):

        self.w1 = self.view.graphicsView.addViewBox()
        self.w1.setAspectLocked(False)
        self.w1.enableAutoRange('xy', True)
        self.view.graphicsView.nextRow()

        self.x_waypoints.append(0)
        self.y_waypoints.append(0)

        self.add_point_set_to_mini_map()
        self.map_point_list[-1].addPoints(self.x_waypoints, self.y_waypoints, size=10, symbol='t', brush='b')

    def handle_pose(self, data):
        if data.gps.FIX_3D:
            if not self.first_point:
                self.first_point = True
                self.dx = data.gps.longitude
                self.dy = data.gps.latitude

            # add (x,y) to tempPose queue
            self.tempPose.put(data)

    def add_point_timeout(self):
        while not self.tempPose.empty():
            pose = self.tempPose.get()

            self.new_x = [(pose.gps.longitude - self.dx)]
            self.new_y = [(pose.gps.latitude - self.dy)]
            self.view.latActual.setText(format_dms(pose.gps.latitude))
            self.view.lonActual.setText(format_dms(pose.gps.longitude))

            quaternion = (
                pose.pose.pose.orientation.x,
                pose.pose.pose.orientation.y,
                pose.pose.pose.orientation.z,
                pose.pose.pose.orientation.w)
            euler = tf.transformations.euler_from_quaternion(quaternion)
            # euler = tf.transformations.euler_from_quaternion(pose.pose.pose.orientation)
            roll = euler[0]
            pitch = euler[1]
            yaw = euler[2]

            self.view.pitchLBL.setText(format_euler_angle(pitch))
            self.view.rollLBL.setText(format_euler_angle(roll))
            self.view.yawLBL.setText(format_euler_angle(yaw))

            self.points_counter += 1
            if self.points_counter % 1000 == 0:
                self.add_point_set_to_mini_map()

            self.map_point_list[-1].addPoints(self.new_x, self.new_y, size=3, symbol='o', brush='w')
            if self.view.zoomGraph.isChecked():
                self.w1.autoRange()

    def add_way_point(self):
        try:
            self.x_waypoints.append(self.new_x[0])
            self.y_waypoints.append(self.new_y[0])
            self.map_point_list[-1].addPoints([self.new_x[0]], [self.new_y[0]], size=10, symbol='t', brush='b')
            if self.view.zoomGraph.isChecked():
                self.w1.autoRange()
        except IndexError:
            rospy.logwarn("Index error, skipping waypoint")
            pass

    def add_coord_dms(self):
        longitude = self.view.lon_deg.value() + self.view.lon_min.value() / 60.0 + self.view.lon_sec.value() / 3600.0
        latitude = self.view.lat_deg.value() + self.view.lat_min.value() / 60.0 + self.view.lat_sec.value() / 3600.0

        if self.view.lat_sign.currentIndex() == 1:
            latitude = - latitude

        if self.view.lon_sign.currentIndex() == 1:
            longitude = - longitude

        x = longitude - self.dx
        y = latitude - self.dy
        self.x_waypoints.append(x)
        self.y_waypoints.append(y)
        self.map_point_list[-1].addPoints([x], [y], size=10, symbol='t', brush='r')
        if self.view.zoomGraph.isChecked():
            self.w1.autoRange()

    def add_coord_dd(self):
        x = self.view.x.value() - self.dx
        y = self.view.y.value() - self.dy
        self.x_waypoints.append(x)
        self.y_waypoints.append(y)
        self.map_point_list[-1].addPoints([x], [y], size=10, symbol='t', brush='r')
        if self.view.zoomGraph.isChecked():
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
