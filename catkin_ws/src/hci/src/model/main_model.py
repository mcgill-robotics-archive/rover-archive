from PyQt5.QtCore import QObject


class MainModel(QObject):
    def __init__(self, parent=None):
        super(MainModel, self).__init__(parent)
