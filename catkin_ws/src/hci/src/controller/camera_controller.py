"""!@brief Global camera controller handles distribution of camera topics to existing screens"""

import rostopic
from PyQt5.QtCore import QObject
from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import QApplication
from sensor_msgs.msg import CompressedImage, Image

from controller.screen_controller import ScreenController


class CameraController(QObject):
    """!@brief Global camera controller

    This class is responsible for acquiring the list of available topics and
    distributing the list to the different screen widgets that are registered.
    The list of available topics is refreshed at each second.

    Example usage:

    @code{.py}
    # Create the controller
    camera_controller = CameraController()

    # Create a view widget
    widget = SingleVideoScreen()

    # Register the video screen
    camera_controller.add_screen(widget)
    @endcode

    When a new screen is registered, a dedicated ScreenController is created
    and associated with the screen.
    """

    def __init__(self, parent=None):
        """!@brief The constructor starts the topic query process

        @param self Python object pointer
        @param parent Qt parent object
        """
        super(CameraController, self).__init__(parent)
        self._screen_list = []
        self._image_type = CompressedImage

        refresh_timer = QTimer(self)
        refresh_timer.setInterval(1000)
        refresh_timer.timeout.connect(self.update_views_topic_list)
        refresh_timer.start()

    def add_screen(self, screen_widget):
        """!@brief Register a new screen widget to the controller

        Creates a dedicated controller for the screen

        @param self Python object pointer
        @param screen_widget The screen widget to register
        """
        topic = screen_widget.get_active_topic()
        if screen_widget is not None:
            screen_controller = ScreenController(screen_widget, topic)
            self._screen_list.append(screen_controller)
            screen_controller.set_type(self._image_type)
            screen_controller.widget.set_feed_list(self.get_topics())

    def set_topic_type(self, image_type=CompressedImage):
        """!@brief Change the type of image we subscribe to.

        Selection between Compressed or Raw Images

        @param self Python object pointer
        @param image_type The message type of the image
        """
        if image_type == Image:
            self._image_type = Image
        else:
            self._image_type = CompressedImage

    def get_topics(self):
        """!@brief Query the ros environment for the list of topics of the
        selected message type.

        The message type must be selected using set_topic_type function prior
        to calling this function.

        @param self Python object pointer
        """
        if self._image_type == CompressedImage:
            topics = rostopic.find_by_type("sensor_msgs/CompressedImage")
        elif self._image_type == Image:
            topics = rostopic.find_by_type("sensor_msgs/Image")
        else:
            topics = []
        return topics

    def update_views_topic_list(self):
        """!@brief Update the list of available topics in all registered
        screen widgets

        @param self Python object pointer
        """
        try:
            topics = self.get_topics()
            for i in self._screen_list:
                i.widget.set_feed_list(topics)
                i.set_type(self._image_type)

        except NameError:
            # topic is not defined, message type is invalid
            pass
        except TypeError:
            # elements in screen list of invalid type, wtf???
            pass
        except rostopic.ROSTopicIOException:
            QApplication.quit()
