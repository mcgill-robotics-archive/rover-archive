#!/usr/bin/env python
# -*- coding: utf-8 -*-

""" Human Computer Interaction. """
import sys

import rospy
from PyQt5.QtWidgets import QApplication
from PyQt5.QtWidgets import QMainWindow

from controller.main_controller import MainController
from model.main_model import MainModel
from views.main_view import MainView

__author__ = "David Lavoie-Boutin"
__version__ = "0.2.0"


class HCI(QMainWindow):
    def __init__(self, parent=None):
        super(HCI, self).__init__(parent)

        rospy.init_node("hci", anonymous=True)
        map_enable = rospy.get_param("~map_enable", False)
        self.main_view = MainView(map_enable, self)
        self.main_controller = MainController(self.main_view, map_enable, self)
        self.main_model = MainModel(self)

        self.setCentralWidget(self.main_view)


def run():
    app = QApplication(sys.argv)

    mainWindow = HCI()
    mainWindow.show()

    sys.exit(app.exec_())


if __name__ == '__main__':
    run()
