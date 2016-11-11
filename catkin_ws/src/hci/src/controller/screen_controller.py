import rospy
from PyQt5.QtCore import QObject, pyqtSlot
from PyQt5.QtGui import QImage
from sensor_msgs.msg import CompressedImage, Image


class ScreenController(QObject):
    def __init__(self, screen_widget=None, topic="", image_type=CompressedImage, parent=None):
        super(ScreenController, self).__init__(parent)
        self.widget = screen_widget
        self.widget.playTopic.connect(self.change_topic)

        self.registered_topic = topic
        self.last_image = QImage()
        self.image_type = image_type
        self.subscriber = None
        self.subscribe()

    def image_callback(self, image):
        if self.image_type == CompressedImage:
            self.last_image = QImage.fromData(image.data)
        elif self.image_type == Image:
            #todo: convert uncompressed image to QImage
            pass

        self.widget.new_sample(self.last_image)

    @pyqtSlot(str)
    def change_topic(self, topic):
        rospy.loginfo("Unregister {0}, change to {1}".format(self.registered_topic, topic))

        if self.subscriber is not None:
            self.subscriber.unregister()

        self.registered_topic = topic
        self.subscribe()

    def subscribe(self):
        if self.registered_topic is not None and self.registered_topic is not "":
            self.subscriber = rospy.Subscriber(self.registered_topic, self.image_type, self.image_callback)

    def screenshot(self, filename):
        if not self.last_image.save(filename):
            rospy.logwarn("Image save unsuccessfull, topic: {0}, file: {1}".format(self.registered_topic, filename))

    def set_type(self, type):
        if type == Image:
            self.image_type = Image
        else:
            self.image_type = CompressedImage
