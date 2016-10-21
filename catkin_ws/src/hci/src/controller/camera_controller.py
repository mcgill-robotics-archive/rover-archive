import rostopic
from PyQt5.QtCore import QObject
from sensor_msgs.msg import CompressedImage, Image

from controller.screen_controller import ScreenController
from views.camera.single_video_screen import SingleVideoScreen


class CameraController(QObject):
    def __init__(self, parent=None):
        super(CameraController, self).__init__(parent)
        self.screen_list = []
        self.image_type = CompressedImage

    def addScreen(self, screenWidget=SingleVideoScreen()):
        topic = screenWidget.get_active_topic()
        if screenWidget is not None:
            screen_controller = ScreenController(screenWidget, topic)
            self.screen_list.append(screen_controller)
            screen_controller.set_type(self.image_type)
            screen_controller.widget.set_feed_list(self.get_topics())

    def set_topic_type(self, type=CompressedImage):
        if type == Image:
            self.image_type = Image
        else:
            self.image_type =  CompressedImage

    def get_topics(self):
        if self.image_type == CompressedImage:
            topics = rostopic.find_by_type("sensor_msgs/CompressedImage")
        elif self.image_type == Image:
            topics = rostopic.find_by_type("sensor_msgs/Image")
        else:
            topics = []
        return topics

    def update_views_topic_list(self, topics):
        try:
            for i in self.screen_list:
                i.widget.set_feed_list(topics)
                i.set_type(type)

        except NameError:
            # topic is not defined, message type is invalid
            pass
        except TypeError:
            # elements in screen list of invalid type, wtf???
            pass
