# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file '../ui_files/MainWindow_V4.ui'
#
# Created: Mon Sep 28 18:58:41 2015
#      by: PyQt4 UI code generator 4.10.4
#
# WARNING! All changes made in this file will be lost!

from PyQt4 import QtCore, QtGui

try:
    _fromUtf8 = QtCore.QString.fromUtf8
except AttributeError:
    def _fromUtf8(s):
        return s

try:
    _encoding = QtGui.QApplication.UnicodeUTF8
    def _translate(context, text, disambig):
        return QtGui.QApplication.translate(context, text, disambig, _encoding)
except AttributeError:
    def _translate(context, text, disambig):
        return QtGui.QApplication.translate(context, text, disambig)

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName(_fromUtf8("MainWindow"))
        MainWindow.resize(1411, 1001)
        self.centralwidget = QtGui.QWidget(MainWindow)
        self.centralwidget.setObjectName(_fromUtf8("centralwidget"))
        self.horizontalLayout_13 = QtGui.QHBoxLayout(self.centralwidget)
        self.horizontalLayout_13.setObjectName(_fromUtf8("horizontalLayout_13"))
        self.verticalLayout_9 = QtGui.QVBoxLayout()
        self.verticalLayout_9.setObjectName(_fromUtf8("verticalLayout_9"))
        self.horizontalLayout_12 = QtGui.QHBoxLayout()
        self.horizontalLayout_12.setObjectName(_fromUtf8("horizontalLayout_12"))
        self.camera2 = QtGui.QLabel(self.centralwidget)
        self.camera2.setMinimumSize(QtCore.QSize(0, 0))
        self.camera2.setSizeIncrement(QtCore.QSize(1, 1))
        self.camera2.setFrameShape(QtGui.QFrame.Box)
        self.camera2.setObjectName(_fromUtf8("camera2"))
        self.horizontalLayout_12.addWidget(self.camera2)
        self.camera3 = QtGui.QLabel(self.centralwidget)
        self.camera3.setMinimumSize(QtCore.QSize(0, 0))
        self.camera3.setSizeIncrement(QtCore.QSize(1, 1))
        self.camera3.setBaseSize(QtCore.QSize(240, 426))
        self.camera3.setFrameShape(QtGui.QFrame.Box)
        self.camera3.setObjectName(_fromUtf8("camera3"))
        self.horizontalLayout_12.addWidget(self.camera3)
        self.verticalLayout_9.addLayout(self.horizontalLayout_12)
        self.camera1 = QtGui.QLabel(self.centralwidget)
        self.camera1.setMinimumSize(QtCore.QSize(0, 0))
        self.camera1.setSizeIncrement(QtCore.QSize(1, 1))
        self.camera1.setFrameShape(QtGui.QFrame.Box)
        self.camera1.setScaledContents(False)
        self.camera1.setIndent(0)
        self.camera1.setObjectName(_fromUtf8("camera1"))
        self.verticalLayout_9.addWidget(self.camera1)
        self.camera_selector = QtGui.QComboBox(self.centralwidget)
        self.camera_selector.setObjectName(_fromUtf8("camera_selector"))
        self.camera_selector.addItem(_fromUtf8(""))
        self.camera_selector.addItem(_fromUtf8(""))
        self.camera_selector.addItem(_fromUtf8(""))
        self.verticalLayout_9.addWidget(self.camera_selector)
        self.groupBox = QtGui.QGroupBox(self.centralwidget)
        self.groupBox.setFlat(True)
        self.groupBox.setObjectName(_fromUtf8("groupBox"))
        self.horizontalLayout_11 = QtGui.QHBoxLayout(self.groupBox)
        self.horizontalLayout_11.setObjectName(_fromUtf8("horizontalLayout_11"))
        self.rot0 = QtGui.QRadioButton(self.groupBox)
        self.rot0.setMaximumSize(QtCore.QSize(16777215, 18))
        self.rot0.setChecked(True)
        self.rot0.setObjectName(_fromUtf8("rot0"))
        self.horizontalLayout_11.addWidget(self.rot0)
        self.rot90 = QtGui.QRadioButton(self.groupBox)
        self.rot90.setMaximumSize(QtCore.QSize(16777215, 18))
        self.rot90.setObjectName(_fromUtf8("rot90"))
        self.horizontalLayout_11.addWidget(self.rot90)
        self.rot180 = QtGui.QRadioButton(self.groupBox)
        self.rot180.setMaximumSize(QtCore.QSize(16777215, 18))
        self.rot180.setObjectName(_fromUtf8("rot180"))
        self.horizontalLayout_11.addWidget(self.rot180)
        self.rot270 = QtGui.QRadioButton(self.groupBox)
        self.rot270.setMaximumSize(QtCore.QSize(16777215, 18))
        self.rot270.setObjectName(_fromUtf8("rot270"))
        self.horizontalLayout_11.addWidget(self.rot270)
        self.verticalLayout_9.addWidget(self.groupBox)
        self.flip_vertical = QtGui.QCheckBox(self.centralwidget)
        self.flip_vertical.setObjectName(_fromUtf8("flip_vertical"))
        self.verticalLayout_9.addWidget(self.flip_vertical)
        self.horizontalLayout_13.addLayout(self.verticalLayout_9)
        self.verticalLayout_8 = QtGui.QVBoxLayout()
        self.verticalLayout_8.setObjectName(_fromUtf8("verticalLayout_8"))
        self.horizontalLayout = QtGui.QHBoxLayout()
        self.horizontalLayout.setObjectName(_fromUtf8("horizontalLayout"))
        self.waypoint = QtGui.QPushButton(self.centralwidget)
        self.waypoint.setObjectName(_fromUtf8("waypoint"))
        self.horizontalLayout.addWidget(self.waypoint)
        self.clearMap = QtGui.QPushButton(self.centralwidget)
        self.clearMap.setObjectName(_fromUtf8("clearMap"))
        self.horizontalLayout.addWidget(self.clearMap)
        self.zoomGraph = QtGui.QCheckBox(self.centralwidget)
        self.zoomGraph.setChecked(True)
        self.zoomGraph.setObjectName(_fromUtf8("zoomGraph"))
        self.horizontalLayout.addWidget(self.zoomGraph)
        self.verticalLayout_8.addLayout(self.horizontalLayout)
        self.tabWidget = QtGui.QTabWidget(self.centralwidget)
        self.tabWidget.setMaximumSize(QtCore.QSize(16777215, 118))
        self.tabWidget.setObjectName(_fromUtf8("tabWidget"))
        self.dms = QtGui.QWidget()
        self.dms.setObjectName(_fromUtf8("dms"))
        self.horizontalLayout_10 = QtGui.QHBoxLayout(self.dms)
        self.horizontalLayout_10.setSpacing(0)
        self.horizontalLayout_10.setMargin(0)
        self.horizontalLayout_10.setObjectName(_fromUtf8("horizontalLayout_10"))
        self.verticalLayout_7 = QtGui.QVBoxLayout()
        self.verticalLayout_7.setObjectName(_fromUtf8("verticalLayout_7"))
        self.label_14 = QtGui.QLabel(self.dms)
        self.label_14.setMaximumSize(QtCore.QSize(16777215, 16))
        self.label_14.setObjectName(_fromUtf8("label_14"))
        self.verticalLayout_7.addWidget(self.label_14)
        self.gridLayout_5 = QtGui.QGridLayout()
        self.gridLayout_5.setObjectName(_fromUtf8("gridLayout_5"))
        self.lat_sign = QtGui.QComboBox(self.dms)
        self.lat_sign.setObjectName(_fromUtf8("lat_sign"))
        self.lat_sign.addItem(_fromUtf8(""))
        self.lat_sign.addItem(_fromUtf8(""))
        self.gridLayout_5.addWidget(self.lat_sign, 0, 0, 1, 1)
        self.lat_deg = QtGui.QSpinBox(self.dms)
        self.lat_deg.setObjectName(_fromUtf8("lat_deg"))
        self.gridLayout_5.addWidget(self.lat_deg, 0, 1, 1, 1)
        self.lat_min = QtGui.QSpinBox(self.dms)
        self.lat_min.setObjectName(_fromUtf8("lat_min"))
        self.gridLayout_5.addWidget(self.lat_min, 1, 0, 1, 1)
        self.lat_sec = QtGui.QDoubleSpinBox(self.dms)
        self.lat_sec.setObjectName(_fromUtf8("lat_sec"))
        self.gridLayout_5.addWidget(self.lat_sec, 1, 1, 1, 1)
        self.verticalLayout_7.addLayout(self.gridLayout_5)
        self.horizontalLayout_10.addLayout(self.verticalLayout_7)
        self.verticalLayout_3 = QtGui.QVBoxLayout()
        self.verticalLayout_3.setObjectName(_fromUtf8("verticalLayout_3"))
        self.label_13 = QtGui.QLabel(self.dms)
        self.label_13.setMaximumSize(QtCore.QSize(16777215, 16))
        self.label_13.setObjectName(_fromUtf8("label_13"))
        self.verticalLayout_3.addWidget(self.label_13)
        self.gridLayout_6 = QtGui.QGridLayout()
        self.gridLayout_6.setObjectName(_fromUtf8("gridLayout_6"))
        self.lon_sign = QtGui.QComboBox(self.dms)
        self.lon_sign.setObjectName(_fromUtf8("lon_sign"))
        self.lon_sign.addItem(_fromUtf8(""))
        self.lon_sign.addItem(_fromUtf8(""))
        self.gridLayout_6.addWidget(self.lon_sign, 0, 0, 1, 1)
        self.lon_deg = QtGui.QSpinBox(self.dms)
        self.lon_deg.setObjectName(_fromUtf8("lon_deg"))
        self.gridLayout_6.addWidget(self.lon_deg, 0, 1, 1, 1)
        self.lon_min = QtGui.QSpinBox(self.dms)
        self.lon_min.setObjectName(_fromUtf8("lon_min"))
        self.gridLayout_6.addWidget(self.lon_min, 1, 0, 1, 1)
        self.lon_sec = QtGui.QDoubleSpinBox(self.dms)
        self.lon_sec.setObjectName(_fromUtf8("lon_sec"))
        self.gridLayout_6.addWidget(self.lon_sec, 1, 1, 1, 1)
        self.verticalLayout_3.addLayout(self.gridLayout_6)
        self.horizontalLayout_10.addLayout(self.verticalLayout_3)
        self.add_waypoint_dms = QtGui.QPushButton(self.dms)
        self.add_waypoint_dms.setObjectName(_fromUtf8("add_waypoint_dms"))
        self.horizontalLayout_10.addWidget(self.add_waypoint_dms)
        self.horizontalLayout_10.setStretch(0, 3)
        self.horizontalLayout_10.setStretch(1, 3)
        self.horizontalLayout_10.setStretch(2, 1)
        self.tabWidget.addTab(self.dms, _fromUtf8(""))
        self.dd = QtGui.QWidget()
        self.dd.setObjectName(_fromUtf8("dd"))
        self.horizontalLayout_5 = QtGui.QHBoxLayout(self.dd)
        self.horizontalLayout_5.setSpacing(0)
        self.horizontalLayout_5.setMargin(0)
        self.horizontalLayout_5.setObjectName(_fromUtf8("horizontalLayout_5"))
        self.gridLayout_4 = QtGui.QGridLayout()
        self.gridLayout_4.setSpacing(0)
        self.gridLayout_4.setObjectName(_fromUtf8("gridLayout_4"))
        self.label_6 = QtGui.QLabel(self.dd)
        self.label_6.setMinimumSize(QtCore.QSize(0, 24))
        self.label_6.setMaximumSize(QtCore.QSize(16777215, 24))
        self.label_6.setObjectName(_fromUtf8("label_6"))
        self.gridLayout_4.addWidget(self.label_6, 0, 0, 1, 1)
        self.label_7 = QtGui.QLabel(self.dd)
        self.label_7.setMinimumSize(QtCore.QSize(0, 24))
        self.label_7.setMaximumSize(QtCore.QSize(16777215, 24))
        self.label_7.setObjectName(_fromUtf8("label_7"))
        self.gridLayout_4.addWidget(self.label_7, 0, 1, 1, 1)
        self.x = QtGui.QDoubleSpinBox(self.dd)
        self.x.setMinimumSize(QtCore.QSize(0, 24))
        self.x.setObjectName(_fromUtf8("x"))
        self.gridLayout_4.addWidget(self.x, 1, 0, 1, 1)
        self.y = QtGui.QDoubleSpinBox(self.dd)
        self.y.setMinimumSize(QtCore.QSize(0, 24))
        self.y.setObjectName(_fromUtf8("y"))
        self.gridLayout_4.addWidget(self.y, 1, 1, 1, 1)
        self.horizontalLayout_5.addLayout(self.gridLayout_4)
        self.add_waypoint_dd = QtGui.QPushButton(self.dd)
        self.add_waypoint_dd.setObjectName(_fromUtf8("add_waypoint_dd"))
        self.horizontalLayout_5.addWidget(self.add_waypoint_dd)
        self.tabWidget.addTab(self.dd, _fromUtf8(""))
        self.verticalLayout_8.addWidget(self.tabWidget)
        self.graphicsView = GraphicsLayoutWidget(self.centralwidget)
        self.graphicsView.setObjectName(_fromUtf8("graphicsView"))
        self.verticalLayout_8.addWidget(self.graphicsView)
        self.label = QtGui.QLabel(self.centralwidget)
        self.label.setObjectName(_fromUtf8("label"))
        self.verticalLayout_8.addWidget(self.label)
        self.horizontalLayout_7 = QtGui.QHBoxLayout()
        self.horizontalLayout_7.setObjectName(_fromUtf8("horizontalLayout_7"))
        self.label_12 = QtGui.QLabel(self.centralwidget)
        self.label_12.setMaximumSize(QtCore.QSize(54, 16777215))
        font = QtGui.QFont()
        font.setPointSize(15)
        font.setBold(True)
        font.setWeight(75)
        self.label_12.setFont(font)
        self.label_12.setObjectName(_fromUtf8("label_12"))
        self.horizontalLayout_7.addWidget(self.label_12)
        self.lonActual = QtGui.QLabel(self.centralwidget)
        font = QtGui.QFont()
        font.setPointSize(12)
        self.lonActual.setFont(font)
        self.lonActual.setObjectName(_fromUtf8("lonActual"))
        self.horizontalLayout_7.addWidget(self.lonActual)
        self.label_15 = QtGui.QLabel(self.centralwidget)
        self.label_15.setMaximumSize(QtCore.QSize(54, 16777215))
        font = QtGui.QFont()
        font.setPointSize(15)
        font.setBold(True)
        font.setWeight(75)
        self.label_15.setFont(font)
        self.label_15.setObjectName(_fromUtf8("label_15"))
        self.horizontalLayout_7.addWidget(self.label_15)
        self.latActual = QtGui.QLabel(self.centralwidget)
        font = QtGui.QFont()
        font.setPointSize(12)
        self.latActual.setFont(font)
        self.latActual.setObjectName(_fromUtf8("latActual"))
        self.horizontalLayout_7.addWidget(self.latActual)
        self.verticalLayout_8.addLayout(self.horizontalLayout_7)
        self.horizontalLayout_9 = QtGui.QHBoxLayout()
        self.horizontalLayout_9.setObjectName(_fromUtf8("horizontalLayout_9"))
        self.label_18 = QtGui.QLabel(self.centralwidget)
        self.label_18.setMaximumSize(QtCore.QSize(18, 16777215))
        font = QtGui.QFont()
        font.setPointSize(15)
        font.setBold(True)
        font.setWeight(75)
        self.label_18.setFont(font)
        self.label_18.setObjectName(_fromUtf8("label_18"))
        self.horizontalLayout_9.addWidget(self.label_18)
        self.pitchLBL = QtGui.QLabel(self.centralwidget)
        self.pitchLBL.setObjectName(_fromUtf8("pitchLBL"))
        self.horizontalLayout_9.addWidget(self.pitchLBL)
        self.label_19 = QtGui.QLabel(self.centralwidget)
        self.label_19.setMaximumSize(QtCore.QSize(18, 16777215))
        font = QtGui.QFont()
        font.setPointSize(15)
        font.setBold(True)
        font.setWeight(75)
        self.label_19.setFont(font)
        self.label_19.setObjectName(_fromUtf8("label_19"))
        self.horizontalLayout_9.addWidget(self.label_19)
        self.rollLBL = QtGui.QLabel(self.centralwidget)
        self.rollLBL.setObjectName(_fromUtf8("rollLBL"))
        self.horizontalLayout_9.addWidget(self.rollLBL)
        self.label_20 = QtGui.QLabel(self.centralwidget)
        self.label_20.setMaximumSize(QtCore.QSize(18, 16777215))
        font = QtGui.QFont()
        font.setPointSize(15)
        font.setBold(True)
        font.setWeight(75)
        self.label_20.setFont(font)
        self.label_20.setObjectName(_fromUtf8("label_20"))
        self.horizontalLayout_9.addWidget(self.label_20)
        self.yawLBL = QtGui.QLabel(self.centralwidget)
        self.yawLBL.setObjectName(_fromUtf8("yawLBL"))
        self.horizontalLayout_9.addWidget(self.yawLBL)
        self.verticalLayout_8.addLayout(self.horizontalLayout_9)
        self.horizontalLayout_13.addLayout(self.verticalLayout_8)
        self.verticalLayout_6 = QtGui.QVBoxLayout()
        self.verticalLayout_6.setObjectName(_fromUtf8("verticalLayout_6"))
        self.horizontalLayout_6 = QtGui.QHBoxLayout()
        self.horizontalLayout_6.setObjectName(_fromUtf8("horizontalLayout_6"))
        self.verticalLayout_5 = QtGui.QVBoxLayout()
        self.verticalLayout_5.setObjectName(_fromUtf8("verticalLayout_5"))
        self.ackMoving = QtGui.QCheckBox(self.centralwidget)
        self.ackMoving.setChecked(True)
        self.ackMoving.setObjectName(_fromUtf8("ackMoving"))
        self.verticalLayout_5.addWidget(self.ackMoving)
        self.screenshot = QtGui.QPushButton(self.centralwidget)
        self.screenshot.setObjectName(_fromUtf8("screenshot"))
        self.verticalLayout_5.addWidget(self.screenshot)
        self.horizontalLayout_6.addLayout(self.verticalLayout_5)
        self.groupBox1 = QtGui.QGroupBox(self.centralwidget)
        self.groupBox1.setFlat(True)
        self.groupBox1.setObjectName(_fromUtf8("groupBox1"))
        self.verticalLayout_2 = QtGui.QVBoxLayout(self.groupBox1)
        self.verticalLayout_2.setObjectName(_fromUtf8("verticalLayout_2"))
        self.ackreman = QtGui.QRadioButton(self.groupBox1)
        self.ackreman.setChecked(True)
        self.ackreman.setObjectName(_fromUtf8("ackreman"))
        self.verticalLayout_2.addWidget(self.ackreman)
        self.skid = QtGui.QRadioButton(self.groupBox1)
        self.skid.setChecked(False)
        self.skid.setObjectName(_fromUtf8("skid"))
        self.verticalLayout_2.addWidget(self.skid)
        self.pointSteer = QtGui.QRadioButton(self.groupBox1)
        self.pointSteer.setObjectName(_fromUtf8("pointSteer"))
        self.verticalLayout_2.addWidget(self.pointSteer)
        self.translatory = QtGui.QRadioButton(self.groupBox1)
        self.translatory.setObjectName(_fromUtf8("translatory"))
        self.verticalLayout_2.addWidget(self.translatory)
        self.horizontalLayout_6.addWidget(self.groupBox1)
        self.verticalLayout_6.addLayout(self.horizontalLayout_6)
        self.horizontalLayout_3 = QtGui.QHBoxLayout()
        self.horizontalLayout_3.setObjectName(_fromUtf8("horizontalLayout_3"))
        self.label_4 = QtGui.QLabel(self.centralwidget)
        self.label_4.setObjectName(_fromUtf8("label_4"))
        self.horizontalLayout_3.addWidget(self.label_4)
        self.sig_qual = QtGui.QLabel(self.centralwidget)
        self.sig_qual.setObjectName(_fromUtf8("sig_qual"))
        self.horizontalLayout_3.addWidget(self.sig_qual)
        self.verticalLayout_6.addLayout(self.horizontalLayout_3)
        self.horizontalLayout_8 = QtGui.QHBoxLayout()
        self.horizontalLayout_8.setObjectName(_fromUtf8("horizontalLayout_8"))
        self.label_5 = QtGui.QLabel(self.centralwidget)
        self.label_5.setObjectName(_fromUtf8("label_5"))
        self.horizontalLayout_8.addWidget(self.label_5)
        self.driveModeSelection = QtGui.QComboBox(self.centralwidget)
        self.driveModeSelection.setObjectName(_fromUtf8("driveModeSelection"))
        self.driveModeSelection.addItem(_fromUtf8(""))
        self.driveModeSelection.addItem(_fromUtf8(""))
        self.driveModeSelection.addItem(_fromUtf8(""))
        self.driveModeSelection.addItem(_fromUtf8(""))
        self.horizontalLayout_8.addWidget(self.driveModeSelection)
        self.verticalLayout_6.addLayout(self.horizontalLayout_8)
        self.gridLayout = QtGui.QGridLayout()
        self.gridLayout.setObjectName(_fromUtf8("gridLayout"))
        self.fl_ok = QtGui.QLabel(self.centralwidget)
        self.fl_ok.setAlignment(QtCore.Qt.AlignCenter)
        self.fl_ok.setObjectName(_fromUtf8("fl_ok"))
        self.gridLayout.addWidget(self.fl_ok, 0, 0, 1, 1)
        self.mr_ok = QtGui.QLabel(self.centralwidget)
        self.mr_ok.setAlignment(QtCore.Qt.AlignCenter)
        self.mr_ok.setObjectName(_fromUtf8("mr_ok"))
        self.gridLayout.addWidget(self.mr_ok, 1, 1, 1, 1)
        self.fr_ok = QtGui.QLabel(self.centralwidget)
        self.fr_ok.setAlignment(QtCore.Qt.AlignCenter)
        self.fr_ok.setObjectName(_fromUtf8("fr_ok"))
        self.gridLayout.addWidget(self.fr_ok, 0, 1, 1, 1)
        self.ml_ok = QtGui.QLabel(self.centralwidget)
        self.ml_ok.setAlignment(QtCore.Qt.AlignCenter)
        self.ml_ok.setObjectName(_fromUtf8("ml_ok"))
        self.gridLayout.addWidget(self.ml_ok, 1, 0, 1, 1)
        self.bl_ok = QtGui.QLabel(self.centralwidget)
        self.bl_ok.setAlignment(QtCore.Qt.AlignCenter)
        self.bl_ok.setObjectName(_fromUtf8("bl_ok"))
        self.gridLayout.addWidget(self.bl_ok, 2, 0, 1, 1)
        self.br_ok = QtGui.QLabel(self.centralwidget)
        self.br_ok.setAlignment(QtCore.Qt.AlignCenter)
        self.br_ok.setObjectName(_fromUtf8("br_ok"))
        self.gridLayout.addWidget(self.br_ok, 2, 1, 1, 1)
        self.verticalLayout_6.addLayout(self.gridLayout)
        self.groupBox_2 = QtGui.QGroupBox(self.centralwidget)
        self.groupBox_2.setObjectName(_fromUtf8("groupBox_2"))
        self.gridLayout_7 = QtGui.QGridLayout(self.groupBox_2)
        self.gridLayout_7.setSpacing(0)
        self.gridLayout_7.setContentsMargins(0, 8, 0, 0)
        self.gridLayout_7.setObjectName(_fromUtf8("gridLayout_7"))
        self.shoulder_motor = QtGui.QRadioButton(self.groupBox_2)
        self.shoulder_motor.setObjectName(_fromUtf8("shoulder_motor"))
        self.gridLayout_7.addWidget(self.shoulder_motor, 0, 0, 1, 1)
        self.wrist_motor = QtGui.QRadioButton(self.groupBox_2)
        self.wrist_motor.setObjectName(_fromUtf8("wrist_motor"))
        self.gridLayout_7.addWidget(self.wrist_motor, 2, 0, 1, 1)
        self.grip_motor = QtGui.QRadioButton(self.groupBox_2)
        self.grip_motor.setObjectName(_fromUtf8("grip_motor"))
        self.gridLayout_7.addWidget(self.grip_motor, 4, 0, 1, 1)
        self.elbow_motor = QtGui.QRadioButton(self.groupBox_2)
        self.elbow_motor.setObjectName(_fromUtf8("elbow_motor"))
        self.gridLayout_7.addWidget(self.elbow_motor, 0, 1, 1, 1)
        self.roll_motor = QtGui.QRadioButton(self.groupBox_2)
        self.roll_motor.setObjectName(_fromUtf8("roll_motor"))
        self.gridLayout_7.addWidget(self.roll_motor, 2, 1, 1, 1)
        self.base_motor = QtGui.QRadioButton(self.groupBox_2)
        self.base_motor.setObjectName(_fromUtf8("base_motor"))
        self.gridLayout_7.addWidget(self.base_motor, 4, 1, 1, 1)
        self.verticalLayout_6.addWidget(self.groupBox_2)
        spacerItem = QtGui.QSpacerItem(20, 40, QtGui.QSizePolicy.Minimum, QtGui.QSizePolicy.Expanding)
        self.verticalLayout_6.addItem(spacerItem)
        self.label_9 = QtGui.QLabel(self.centralwidget)
        self.label_9.setObjectName(_fromUtf8("label_9"))
        self.verticalLayout_6.addWidget(self.label_9)
        self.gridLayout_2 = QtGui.QGridLayout()
        self.gridLayout_2.setObjectName(_fromUtf8("gridLayout_2"))
        self.tr2 = QtGui.QPushButton(self.centralwidget)
        self.tr2.setObjectName(_fromUtf8("tr2"))
        self.gridLayout_2.addWidget(self.tr2, 4, 0, 1, 1)
        self.tr1 = QtGui.QPushButton(self.centralwidget)
        self.tr1.setObjectName(_fromUtf8("tr1"))
        self.gridLayout_2.addWidget(self.tr1, 3, 0, 1, 1)
        self.tr3 = QtGui.QPushButton(self.centralwidget)
        self.tr3.setObjectName(_fromUtf8("tr3"))
        self.gridLayout_2.addWidget(self.tr3, 5, 0, 1, 1)
        self.tr4 = QtGui.QPushButton(self.centralwidget)
        self.tr4.setObjectName(_fromUtf8("tr4"))
        self.gridLayout_2.addWidget(self.tr4, 3, 1, 1, 1)
        self.tr5 = QtGui.QPushButton(self.centralwidget)
        self.tr5.setObjectName(_fromUtf8("tr5"))
        self.gridLayout_2.addWidget(self.tr5, 4, 1, 1, 1)
        self.tr6 = QtGui.QPushButton(self.centralwidget)
        self.tr6.setObjectName(_fromUtf8("tr6"))
        self.gridLayout_2.addWidget(self.tr6, 5, 1, 1, 1)
        self.tr7 = QtGui.QPushButton(self.centralwidget)
        self.tr7.setObjectName(_fromUtf8("tr7"))
        self.gridLayout_2.addWidget(self.tr7, 6, 1, 1, 1)
        self.verticalLayout_6.addLayout(self.gridLayout_2)
        self.frame = QtGui.QFrame(self.centralwidget)
        self.frame.setFrameShape(QtGui.QFrame.StyledPanel)
        self.frame.setFrameShadow(QtGui.QFrame.Raised)
        self.frame.setObjectName(_fromUtf8("frame"))
        self.verticalLayout_4 = QtGui.QVBoxLayout(self.frame)
        self.verticalLayout_4.setSpacing(0)
        self.verticalLayout_4.setMargin(0)
        self.verticalLayout_4.setObjectName(_fromUtf8("verticalLayout_4"))
        self.label_3 = QtGui.QLabel(self.frame)
        self.label_3.setAlignment(QtCore.Qt.AlignCenter)
        self.label_3.setObjectName(_fromUtf8("label_3"))
        self.verticalLayout_4.addWidget(self.label_3)
        self.line = QtGui.QFrame(self.frame)
        self.line.setFrameShape(QtGui.QFrame.HLine)
        self.line.setFrameShadow(QtGui.QFrame.Sunken)
        self.line.setObjectName(_fromUtf8("line"))
        self.verticalLayout_4.addWidget(self.line)
        self.gridLayout_3 = QtGui.QGridLayout()
        self.gridLayout_3.setObjectName(_fromUtf8("gridLayout_3"))
        self.DriveMode = QtGui.QPushButton(self.frame)
        self.DriveMode.setCheckable(True)
        self.DriveMode.setObjectName(_fromUtf8("DriveMode"))
        self.gridLayout_3.addWidget(self.DriveMode, 0, 0, 1, 1)
        self.EndEffectorMode = QtGui.QPushButton(self.frame)
        self.EndEffectorMode.setEnabled(False)
        self.EndEffectorMode.setText(_fromUtf8(""))
        self.EndEffectorMode.setCheckable(True)
        self.EndEffectorMode.setChecked(False)
        self.EndEffectorMode.setObjectName(_fromUtf8("EndEffectorMode"))
        self.gridLayout_3.addWidget(self.EndEffectorMode, 0, 1, 1, 1)
        self.ArmBaseMode = QtGui.QPushButton(self.frame)
        self.ArmBaseMode.setCheckable(True)
        self.ArmBaseMode.setChecked(False)
        self.ArmBaseMode.setObjectName(_fromUtf8("ArmBaseMode"))
        self.gridLayout_3.addWidget(self.ArmBaseMode, 1, 0, 1, 1)
        self.function4 = QtGui.QPushButton(self.frame)
        self.function4.setCheckable(True)
        self.function4.setChecked(False)
        self.function4.setObjectName(_fromUtf8("function4"))
        self.gridLayout_3.addWidget(self.function4, 1, 1, 1, 1)
        self.verticalLayout_4.addLayout(self.gridLayout_3)
        self.verticalLayout_6.addWidget(self.frame)
        self.frame_2 = QtGui.QFrame(self.centralwidget)
        self.frame_2.setFrameShape(QtGui.QFrame.StyledPanel)
        self.frame_2.setFrameShadow(QtGui.QFrame.Raised)
        self.frame_2.setObjectName(_fromUtf8("frame_2"))
        self.verticalLayout = QtGui.QVBoxLayout(self.frame_2)
        self.verticalLayout.setSpacing(0)
        self.verticalLayout.setMargin(0)
        self.verticalLayout.setObjectName(_fromUtf8("verticalLayout"))
        self.horizontalLayout_2 = QtGui.QHBoxLayout()
        self.horizontalLayout_2.setObjectName(_fromUtf8("horizontalLayout_2"))
        self.label_2 = QtGui.QLabel(self.frame_2)
        self.label_2.setMaximumSize(QtCore.QSize(100, 16777215))
        self.label_2.setObjectName(_fromUtf8("label_2"))
        self.horizontalLayout_2.addWidget(self.label_2)
        self.arm_mode = QtGui.QComboBox(self.frame_2)
        self.arm_mode.setObjectName(_fromUtf8("arm_mode"))
        self.arm_mode.addItem(_fromUtf8(""))
        self.arm_mode.addItem(_fromUtf8(""))
        self.horizontalLayout_2.addWidget(self.arm_mode)
        self.verticalLayout.addLayout(self.horizontalLayout_2)
        self.horizontalLayout_4 = QtGui.QHBoxLayout()
        self.horizontalLayout_4.setObjectName(_fromUtf8("horizontalLayout_4"))
        self.label_8 = QtGui.QLabel(self.frame_2)
        self.label_8.setMaximumSize(QtCore.QSize(120, 16777215))
        self.label_8.setObjectName(_fromUtf8("label_8"))
        self.horizontalLayout_4.addWidget(self.label_8)
        self.coordinateSystem = QtGui.QComboBox(self.frame_2)
        self.coordinateSystem.setObjectName(_fromUtf8("coordinateSystem"))
        self.coordinateSystem.addItem(_fromUtf8(""))
        self.coordinateSystem.addItem(_fromUtf8(""))
        self.horizontalLayout_4.addWidget(self.coordinateSystem)
        self.verticalLayout.addLayout(self.horizontalLayout_4)
        self.verticalLayout_6.addWidget(self.frame_2)
        self.horizontalLayout_14 = QtGui.QHBoxLayout()
        self.horizontalLayout_14.setObjectName(_fromUtf8("horizontalLayout_14"))
        self.read_voltage_button = QtGui.QPushButton(self.centralwidget)
        self.read_voltage_button.setObjectName(_fromUtf8("read_voltage_button"))
        self.horizontalLayout_14.addWidget(self.read_voltage_button)
        self.input_voltage_label = QtGui.QLabel(self.centralwidget)
        self.input_voltage_label.setObjectName(_fromUtf8("input_voltage_label"))
        self.horizontalLayout_14.addWidget(self.input_voltage_label)
        self.verticalLayout_6.addLayout(self.horizontalLayout_14)
        self.horizontalLayout_13.addLayout(self.verticalLayout_6)
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtGui.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 1411, 22))
        self.menubar.setObjectName(_fromUtf8("menubar"))
        self.menuFile = QtGui.QMenu(self.menubar)
        self.menuFile.setObjectName(_fromUtf8("menuFile"))
        MainWindow.setMenuBar(self.menubar)
        self.toolBar = QtGui.QToolBar(MainWindow)
        self.toolBar.setObjectName(_fromUtf8("toolBar"))
        MainWindow.addToolBar(QtCore.Qt.TopToolBarArea, self.toolBar)
        self.actionQuit = QtGui.QAction(MainWindow)
        self.actionQuit.setObjectName(_fromUtf8("actionQuit"))
        self.actionRestart = QtGui.QAction(MainWindow)
        self.actionRestart.setObjectName(_fromUtf8("actionRestart"))
        self.menuFile.addAction(self.actionRestart)
        self.menuFile.addAction(self.actionQuit)
        self.menubar.addAction(self.menuFile.menuAction())

        self.retranslateUi(MainWindow)
        self.tabWidget.setCurrentIndex(0)
        QtCore.QObject.connect(self.actionQuit, QtCore.SIGNAL(_fromUtf8("triggered()")), MainWindow.close)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)
        MainWindow.setTabOrder(self.waypoint, self.clearMap)
        MainWindow.setTabOrder(self.clearMap, self.ackMoving)
        MainWindow.setTabOrder(self.ackMoving, self.ackreman)
        MainWindow.setTabOrder(self.ackreman, self.skid)
        MainWindow.setTabOrder(self.skid, self.pointSteer)
        MainWindow.setTabOrder(self.pointSteer, self.translatory)
        MainWindow.setTabOrder(self.translatory, self.screenshot)
        MainWindow.setTabOrder(self.screenshot, self.driveModeSelection)
        MainWindow.setTabOrder(self.driveModeSelection, self.DriveMode)
        MainWindow.setTabOrder(self.DriveMode, self.EndEffectorMode)
        MainWindow.setTabOrder(self.EndEffectorMode, self.ArmBaseMode)
        MainWindow.setTabOrder(self.ArmBaseMode, self.function4)
        MainWindow.setTabOrder(self.function4, self.arm_mode)
        MainWindow.setTabOrder(self.arm_mode, self.coordinateSystem)
        MainWindow.setTabOrder(self.coordinateSystem, self.zoomGraph)
        MainWindow.setTabOrder(self.zoomGraph, self.tabWidget)
        MainWindow.setTabOrder(self.tabWidget, self.x)
        MainWindow.setTabOrder(self.x, self.y)
        MainWindow.setTabOrder(self.y, self.add_waypoint_dd)
        MainWindow.setTabOrder(self.add_waypoint_dd, self.lat_sign)
        MainWindow.setTabOrder(self.lat_sign, self.lat_deg)
        MainWindow.setTabOrder(self.lat_deg, self.lat_min)
        MainWindow.setTabOrder(self.lat_min, self.lat_sec)
        MainWindow.setTabOrder(self.lat_sec, self.lon_sign)
        MainWindow.setTabOrder(self.lon_sign, self.lon_deg)
        MainWindow.setTabOrder(self.lon_deg, self.lon_min)
        MainWindow.setTabOrder(self.lon_min, self.lon_sec)
        MainWindow.setTabOrder(self.lon_sec, self.graphicsView)

    def retranslateUi(self, MainWindow):
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow", None))
        self.camera2.setText(_translate("MainWindow", "camLeft", None))
        self.camera3.setText(_translate("MainWindow", "camRight", None))
        self.camera1.setText(_translate("MainWindow", "camMain", None))
        self.camera_selector.setItemText(0, _translate("MainWindow", "camera/pan_tilt", None))
        self.camera_selector.setItemText(1, _translate("MainWindow", "camera/wide_angle", None))
        self.camera_selector.setItemText(2, _translate("MainWindow", "camera/arm", None))
        self.rot0.setText(_translate("MainWindow", "0", None))
        self.rot90.setText(_translate("MainWindow", "90", None))
        self.rot180.setText(_translate("MainWindow", "180", None))
        self.rot270.setText(_translate("MainWindow", "270", None))
        self.flip_vertical.setText(_translate("MainWindow", "Flip vertical", None))
        self.waypoint.setText(_translate("MainWindow", "Mark Current as Waypoint", None))
        self.clearMap.setText(_translate("MainWindow", "Clear", None))
        self.zoomGraph.setText(_translate("MainWindow", "Zoom graph", None))
        self.label_14.setText(_translate("MainWindow", "Latitude (y)", None))
        self.lat_sign.setItemText(0, _translate("MainWindow", "N", None))
        self.lat_sign.setItemText(1, _translate("MainWindow", "S", None))
        self.label_13.setText(_translate("MainWindow", "Longitude (x)", None))
        self.lon_sign.setItemText(0, _translate("MainWindow", "E", None))
        self.lon_sign.setItemText(1, _translate("MainWindow", "W", None))
        self.add_waypoint_dms.setText(_translate("MainWindow", "Add Waypoint", None))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.dms), _translate("MainWindow", "DMS", None))
        self.label_6.setText(_translate("MainWindow", "Lon", None))
        self.label_7.setText(_translate("MainWindow", "Lat", None))
        self.add_waypoint_dd.setText(_translate("MainWindow", "Add Waypoint", None))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.dd), _translate("MainWindow", "DD", None))
        self.label.setText(_translate("MainWindow", "Current position", None))
        self.label_12.setText(_translate("MainWindow", "Lon", None))
        self.lonActual.setText(_translate("MainWindow", "0.00", None))
        self.label_15.setText(_translate("MainWindow", "Lat", None))
        self.latActual.setText(_translate("MainWindow", "0.00", None))
        self.label_18.setText(_translate("MainWindow", "P", None))
        self.pitchLBL.setText(_translate("MainWindow", "Pitch", None))
        self.label_19.setText(_translate("MainWindow", "R", None))
        self.rollLBL.setText(_translate("MainWindow", "Roll", None))
        self.label_20.setText(_translate("MainWindow", "Y", None))
        self.yawLBL.setText(_translate("MainWindow", "Yaw", None))
        self.ackMoving.setText(_translate("MainWindow", "Motor Enable", None))
        self.screenshot.setText(_translate("MainWindow", "Capture cam", None))
        self.ackreman.setText(_translate("MainWindow", "Ackreman", None))
        self.skid.setText(_translate("MainWindow", "Skid", None))
        self.pointSteer.setText(_translate("MainWindow", "Point steer ?", None))
        self.translatory.setText(_translate("MainWindow", "Translatory", None))
        self.label_4.setText(_translate("MainWindow", "Signal quality", None))
        self.sig_qual.setText(_translate("MainWindow", "123 ms", None))
        self.label_5.setText(_translate("MainWindow", "Drive motor mode", None))
        self.driveModeSelection.setItemText(0, _translate("MainWindow", "Low Speed", None))
        self.driveModeSelection.setItemText(1, _translate("MainWindow", "Med Speed", None))
        self.driveModeSelection.setItemText(2, _translate("MainWindow", "High Speed", None))
        self.driveModeSelection.setItemText(3, _translate("MainWindow", "Open Loop", None))
        self.fl_ok.setText(_translate("MainWindow", "Ok", None))
        self.mr_ok.setText(_translate("MainWindow", "Ok", None))
        self.fr_ok.setText(_translate("MainWindow", "Ok", None))
        self.ml_ok.setText(_translate("MainWindow", "Ok", None))
        self.bl_ok.setText(_translate("MainWindow", "Ok", None))
        self.br_ok.setText(_translate("MainWindow", "Ok", None))
        self.groupBox_2.setTitle(_translate("MainWindow", "Arm Motor", None))
        self.shoulder_motor.setText(_translate("MainWindow", "Shoulder", None))
        self.wrist_motor.setText(_translate("MainWindow", "Wrist", None))
        self.grip_motor.setText(_translate("MainWindow", "Grip", None))
        self.elbow_motor.setText(_translate("MainWindow", "Elbow", None))
        self.roll_motor.setText(_translate("MainWindow", "Roll", None))
        self.base_motor.setText(_translate("MainWindow", "Base", None))
        self.label_9.setText(_translate("MainWindow", "Science box", None))
        self.tr2.setText(_translate("MainWindow", "Tray 2", None))
        self.tr1.setText(_translate("MainWindow", "Tray 1", None))
        self.tr3.setText(_translate("MainWindow", "Tray 3", None))
        self.tr4.setText(_translate("MainWindow", "Tray 4", None))
        self.tr5.setText(_translate("MainWindow", "Tray 5", None))
        self.tr6.setText(_translate("MainWindow", "Tray 6", None))
        self.tr7.setText(_translate("MainWindow", "Tray 7", None))
        self.label_3.setText(_translate("MainWindow", "Joystick Control Function", None))
        self.DriveMode.setText(_translate("MainWindow", "Drive", None))
        self.ArmBaseMode.setText(_translate("MainWindow", "Arm Mode", None))
        self.function4.setText(_translate("MainWindow", "Camera", None))
        self.label_2.setText(_translate("MainWindow", "Arm mode", None))
        self.arm_mode.setItemText(0, _translate("MainWindow", "Position Control", None))
        self.arm_mode.setItemText(1, _translate("MainWindow", "Velocity Control", None))
        self.label_8.setText(_translate("MainWindow", "Coordinate", None))
        self.coordinateSystem.setItemText(0, _translate("MainWindow", "Cylindrical", None))
        self.coordinateSystem.setItemText(1, _translate("MainWindow", "Cartesian", None))
        self.read_voltage_button.setText(_translate("MainWindow", "Read Voltage", None))
        self.input_voltage_label.setText(_translate("MainWindow", "Input Voltage", None))
        self.menuFile.setTitle(_translate("MainWindow", "File", None))
        self.toolBar.setWindowTitle(_translate("MainWindow", "toolBar", None))
        self.actionQuit.setText(_translate("MainWindow", "Quit", None))
        self.actionRestart.setText(_translate("MainWindow", "Restart", None))

from pyqtgraph import GraphicsLayoutWidget
