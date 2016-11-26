"""!@brief Map module using Scatter Plot Items from pyqtgraph library"""
import sys

from PyQt5.QtCore import pyqtSlot
from PyQt5.QtWidgets import QApplication
from PyQt5.QtWidgets import QHBoxLayout
from PyQt5.QtWidgets import QWidget
from pyqtgraph import GraphicsLayoutWidget, ScatterPlotItem


class Coordinate(object):
    """!@brief Simple structure like class to record coordinates"""
    def __init__(self, x, y):
        self._x = x
        self._y = y

    def get_x(self):
        return self._x

    def get_y(self):
        return self._y


class Map(QWidget):
    """!@brief Display widget with the Scatter Plot item and the control logic
    to display points and waypoints"""

    def __init__(self, parent=None):
        """!@brief Constructor layout graph in window and add first plot items

        @param self Python object pointer
        @param parent Qt hierarchy model
        """
        super(Map, self).__init__(parent)

        self._graph_layout = GraphicsLayoutWidget(self)

        layout = QHBoxLayout()
        layout.addWidget(self._graph_layout)
        layout.setContentsMargins(0, 0, 0, 0)
        self.setLayout(layout)

        self._point_counter = 0

        self._plot_items = []
        self._waypoint_list = []
        self._center_point = None

        self._viewbox = self._graph_layout.addViewBox()
        self._viewbox.setAspectLocked(True)
        self._viewbox.enableAutoRange('xy', True)
        self._waypoint_plot = ScatterPlotItem(size=20, pen='b', symbol='t')
        self._viewbox.addItem(self._waypoint_plot)
        self._add_point_set()

        self.add_waypoint(0, 0)

    def _add_point_set(self):
        """!@brief Create a new plot item and add it to the window

        ScatterPlotItems stores data in numpy array, which implements the
        append function with a copy, this gets highly inefficient with large
        data set, so we split the data in 1000 sample sets.

        @param self Python object pointer
        """
        plot_item = ScatterPlotItem(size=10, pen='w', pxMode=True)
        self._plot_items.append(plot_item)
        self._viewbox.addItem(plot_item)
        pass

    @pyqtSlot(float, float)
    def add_waypoint(self, x, y):
        """!@brief Add a permanent waypoint marker to the window

        @param x the x position
        @param y the y position
        @param self Python object pointer
        """
        if self._center_point is None:
            self._waypoint_plot.addPoints([x], [y])
        else:
            self._waypoint_plot.addPoints([x - self._center_point.get_x()],
                                          [y - self._center_point.get_y()])

    @pyqtSlot(float, float)
    def add_point(self, x, y):
        """!@brief Add a single data point to the plot

        @param x the x position
        @param y the y position
        @param self Python object pointer
        """
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
        """!@brief Remove the data points from the display

        The waypoints remain as well as the initialisation value

        @param self Python object pointer
        """
        for item in self._plot_items:
            self._viewbox.removeItem(item)
            self._plot_items.remove(item)

        self._add_point_set()


if __name__ == '__main__':
    app = QApplication(sys.argv)
    map_widget = Map()
    map_widget.show()
    map_widget.add_point(1, 0)
    map_widget.clear_points()
    map_widget.setGeometry(100, 100, 800, 600)
    sys.exit(app.exec_())
