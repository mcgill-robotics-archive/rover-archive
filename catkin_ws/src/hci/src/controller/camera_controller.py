import rostopic
from PyQt5.QtCore import QObject
from PyQt5.QtCore import QTimer
from sensor_msgs.msg import CompressedImage, Image

from controller.screen_controller import ScreenController


class CameraController(QObject):
    def __init__(self, parent=None):
        super(CameraController, self).__init__(parent)
        self.screen_list = []
        self.image_type = CompressedImage

        refresh_timer = QTimer(self)
        refresh_timer.setInterval(1000)
        refresh_timer.timeout.connect(self.update_views_topic_list)
        refresh_timer.start()

    def add_screen(self, screen_widget=None):
        topic = screen_widget.get_active_topic()
        if screen_widget is not None:
            screen_controller = ScreenController(screen_widget, topic)
            self.screen_list.append(screen_controller)
            screen_controller.set_type(self.image_type)
            screen_controller.widget.set_feed_list(self.get_topics())

    def set_topic_type(self, image_type=CompressedImage):
        if image_type == Image:
            self.image_type = Image
        else:
            self.image_type = CompressedImage

    def get_topics(self):
        if self.image_type == CompressedImage:
            topics = rostopic.find_by_type("sensor_msgs/CompressedImage")
        elif self.image_type == Image:
            topics = rostopic.find_by_type("sensor_msgs/Image")
        else:
            topics = []
        return topics

    def update_views_topic_list(self):

        topics = self.get_topics()
        try:
            for i in self.screen_list:
                i.widget.set_feed_list(topics)
                i.set_type(self.image_type)

        except NameError:
            # topic is not defined, message type is invalid
            pass
        except TypeError:
            # elements in screen list of invalid type, wtf???
            pass
