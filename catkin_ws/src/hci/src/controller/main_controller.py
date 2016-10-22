from PyQt5.QtCore import QObject


class MainController(QObject):
    def __init__(self, parent=None):
        super(MainController, self).__init__(parent)
