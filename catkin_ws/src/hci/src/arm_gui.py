# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'ArmGui.ui'
#
# Created: Tue May 31 17:35:44 2016
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
        MainWindow.resize(994, 779)
        self.centralwidget = QtGui.QWidget(MainWindow)
        self.centralwidget.setObjectName(_fromUtf8("centralwidget"))
        self.verticalLayout_3 = QtGui.QVBoxLayout(self.centralwidget)
        self.verticalLayout_3.setObjectName(_fromUtf8("verticalLayout_3"))
        self.horizontalLayout = QtGui.QHBoxLayout()
        self.horizontalLayout.setObjectName(_fromUtf8("horizontalLayout"))
        self.camera2 = QtGui.QLabel(self.centralwidget)
        self.camera2.setMinimumSize(QtCore.QSize(0, 0))
        self.camera2.setSizeIncrement(QtCore.QSize(1, 1))
        self.camera2.setFrameShape(QtGui.QFrame.Box)
        self.camera2.setObjectName(_fromUtf8("camera2"))
        self.horizontalLayout.addWidget(self.camera2)
        self.camera3 = QtGui.QLabel(self.centralwidget)
        self.camera3.setMinimumSize(QtCore.QSize(0, 0))
        self.camera3.setSizeIncrement(QtCore.QSize(1, 1))
        self.camera3.setBaseSize(QtCore.QSize(240, 426))
        self.camera3.setFrameShape(QtGui.QFrame.Box)
        self.camera3.setObjectName(_fromUtf8("camera3"))
        self.horizontalLayout.addWidget(self.camera3)
        self.verticalLayout_3.addLayout(self.horizontalLayout)
        self.horizontalLayout_2 = QtGui.QHBoxLayout()
        self.horizontalLayout_2.setObjectName(_fromUtf8("horizontalLayout_2"))
        self.verticalLayout_2 = QtGui.QVBoxLayout()
        self.verticalLayout_2.setObjectName(_fromUtf8("verticalLayout_2"))
        self.camera1 = QtGui.QLabel(self.centralwidget)
        self.camera1.setMinimumSize(QtCore.QSize(0, 0))
        self.camera1.setSizeIncrement(QtCore.QSize(1, 1))
        self.camera1.setFrameShape(QtGui.QFrame.Box)
        self.camera1.setScaledContents(False)
        self.camera1.setIndent(0)
        self.camera1.setObjectName(_fromUtf8("camera1"))
        self.verticalLayout_2.addWidget(self.camera1)
        self.horizontalLayout_2.addLayout(self.verticalLayout_2)
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
        self.augurDrillEnable = QtGui.QCheckBox(self.centralwidget)
        self.augurDrillEnable.setObjectName(_fromUtf8("augurDrillEnable"))
        self.verticalLayout_5.addWidget(self.augurDrillEnable)
        self.horizontalLayout_6.addLayout(self.verticalLayout_5)
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
        spacerItem = QtGui.QSpacerItem(20, 40, QtGui.QSizePolicy.Minimum, QtGui.QSizePolicy.Expanding)
        self.verticalLayout_6.addItem(spacerItem)
        self.groupBox_2 = QtGui.QGroupBox(self.centralwidget)
        self.groupBox_2.setTitle(_fromUtf8(""))
        self.groupBox_2.setFlat(True)
        self.groupBox_2.setObjectName(_fromUtf8("groupBox_2"))
        self.gridLayout_2 = QtGui.QGridLayout(self.groupBox_2)
        self.gridLayout_2.setMargin(0)
        self.gridLayout_2.setHorizontalSpacing(0)
        self.gridLayout_2.setVerticalSpacing(4)
        self.gridLayout_2.setObjectName(_fromUtf8("gridLayout_2"))
        self.end_eff = QtGui.QRadioButton(self.groupBox_2)
        self.end_eff.setAutoExclusive(True)
        self.end_eff.setObjectName(_fromUtf8("end_eff"))
        self.gridLayout_2.addWidget(self.end_eff, 0, 1, 1, 1)
        self.base = QtGui.QRadioButton(self.groupBox_2)
        self.base.setObjectName(_fromUtf8("base"))
        self.gridLayout_2.addWidget(self.base, 0, 0, 1, 1)
        self.diff2 = QtGui.QRadioButton(self.groupBox_2)
        self.diff2.setObjectName(_fromUtf8("diff2"))
        self.gridLayout_2.addWidget(self.diff2, 1, 1, 1, 1)
        self.diff1 = QtGui.QRadioButton(self.groupBox_2)
        self.diff1.setObjectName(_fromUtf8("diff1"))
        self.gridLayout_2.addWidget(self.diff1, 1, 0, 1, 1)
        self.verticalLayout_6.addWidget(self.groupBox_2)
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
        self.gridLayout_3 = QtGui.QGridLayout()
        self.gridLayout_3.setObjectName(_fromUtf8("gridLayout_3"))
        self.ArmBaseMode = QtGui.QPushButton(self.frame)
        self.ArmBaseMode.setEnabled(True)
        self.ArmBaseMode.setCheckable(True)
        self.ArmBaseMode.setChecked(False)
        self.ArmBaseMode.setObjectName(_fromUtf8("ArmBaseMode"))
        self.gridLayout_3.addWidget(self.ArmBaseMode, 0, 0, 1, 1)
        self.function4 = QtGui.QPushButton(self.frame)
        self.function4.setCheckable(True)
        self.function4.setChecked(False)
        self.function4.setObjectName(_fromUtf8("function4"))
        self.gridLayout_3.addWidget(self.function4, 0, 1, 1, 1)
        self.verticalLayout_4.addLayout(self.gridLayout_3)
        self.verticalLayout_6.addWidget(self.frame)
        self.horizontalLayout_14 = QtGui.QHBoxLayout()
        self.horizontalLayout_14.setObjectName(_fromUtf8("horizontalLayout_14"))
        self.verticalLayout_6.addLayout(self.horizontalLayout_14)
        self.horizontalLayout_2.addLayout(self.verticalLayout_6)
        self.verticalLayout_3.addLayout(self.horizontalLayout_2)
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtGui.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 994, 25))
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
        QtCore.QObject.connect(self.actionQuit, QtCore.SIGNAL(_fromUtf8("triggered()")), MainWindow.close)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)
        MainWindow.setTabOrder(self.ackMoving, self.ArmBaseMode)
        MainWindow.setTabOrder(self.ArmBaseMode, self.function4)

    def retranslateUi(self, MainWindow):
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow", None))
        self.camera2.setText(_translate("MainWindow", "camLeft", None))
        self.camera3.setText(_translate("MainWindow", "camRight", None))
        self.camera1.setText(_translate("MainWindow", "camMain", None))
        self.ackMoving.setText(_translate("MainWindow", "Motor Enable", None))
        self.augurDrillEnable.setText(_translate("MainWindow", "Augur Drill", None))
        self.label_4.setText(_translate("MainWindow", "Signal quality", None))
        self.sig_qual.setText(_translate("MainWindow", "123 ms", None))
        self.fl_ok.setText(_translate("MainWindow", "Ok", None))
        self.mr_ok.setText(_translate("MainWindow", "Ok", None))
        self.fr_ok.setText(_translate("MainWindow", "Ok", None))
        self.ml_ok.setText(_translate("MainWindow", "Ok", None))
        self.bl_ok.setText(_translate("MainWindow", "Ok", None))
        self.br_ok.setText(_translate("MainWindow", "Ok", None))
        self.end_eff.setText(_translate("MainWindow", "End Effector", None))
        self.base.setText(_translate("MainWindow", "Base", None))
        self.diff2.setText(_translate("MainWindow", "Differential 2", None))
        self.diff1.setText(_translate("MainWindow", "Differential 1", None))
        self.label_3.setText(_translate("MainWindow", "Joystick Control Function", None))
        self.ArmBaseMode.setText(_translate("MainWindow", "Arm", None))
        self.function4.setText(_translate("MainWindow", "Camera", None))
        self.menuFile.setTitle(_translate("MainWindow", "File", None))
        self.toolBar.setWindowTitle(_translate("MainWindow", "toolBar", None))
        self.actionQuit.setText(_translate("MainWindow", "Quit", None))
        self.actionRestart.setText(_translate("MainWindow", "Restart", None))

