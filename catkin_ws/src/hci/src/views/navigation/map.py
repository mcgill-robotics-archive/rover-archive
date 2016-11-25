import sys

from PyQt5.QtCore import pyqtSlot
from PyQt5.QtWidgets import QApplication
from PyQt5.QtWidgets import QHBoxLayout
from PyQt5.QtWidgets import QWidget
from pyqtgraph import GraphicsLayoutWidget, ScatterPlotItem


class Coordinate(object):
    def __init__(self, x, y):
        self._x = x
        self._y = y

    def get_x(self):
        return self._x

    def get_y(self):
        return self._y


class Map(QWidget):
    def __init__(self, parent=None):
        super(Map, self).__init__(parent)

        self.graph_layout = GraphicsLayoutWidget(self)

        layout = QHBoxLayout()
        layout.addWidget(self.graph_layout)
        layout.setContentsMargins(0, 0, 0, 0)
        self.setLayout(layout)

        self._point_counter = 0

        self._plot_items = []
        self._waypoint_list = []
        self._center_point = None

        self._viewbox = self.graph_layout.addViewBox()
        self._viewbox.setAspectLocked(True)
        self._viewbox.enableAutoRange('xy', True)
        self._waypoint_plot = ScatterPlotItem(size=20, pen='b', symbol='t')
        self._viewbox.addItem(self._waypoint_plot)
        self._add_point_set()

        self.add_waypoint(0, 0)

    def _add_point_set(self):
        plot_item = ScatterPlotItem(size=10, pen='w', pxMode=True)
        self._plot_items.append(plot_item)
        self._viewbox.addItem(plot_item)
        pass

    @pyqtSlot(float, float)
    def add_waypoint(self, x, y):
        if self._center_point is None:
            self._waypoint_plot.addPoints([x], [y])
        else:
            self._waypoint_plot.addPoints([x - self._center_point.get_x()],
                                          [y - self._center_point.get_y()])

    @pyqtSlot(float, float)
    def add_point(self, x, y):
        if self._point_counter > 1000:
            self._add_point_set()

        if self._center_point is None:
            self._center_point = Coordinate(x, y)

        self._plot_items[-1].addPoints([x - self._center_point.get_x()],
                                       [y - self._center_point.get_y()],
                                       size=3, symbol='o', bursh='w')
        self._point_counter += 1

    @pyqtSlot()
    def clear_points(self):
        for item in self._plot_items:
            self._viewbox.removeItem(item)
            self._plot_items.remove(item)

        self._add_point_set()
        self._center_point = None


if __name__ == '__main__':
    app = QApplication(sys.argv)
    map_widget = Map()
    map_widget.show()
    map_widget.add_point(1, 0)
    map_widget.clear_points()
    map_widget.setGeometry(100, 100, 800, 600)
    sys.exit(app.exec_())
