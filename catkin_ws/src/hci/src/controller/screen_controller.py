import rospy
from PyQt5.QtCore import QObject, pyqtSlot
from PyQt5.QtGui import QImage
from sensor_msgs.msg import CompressedImage, Image

from views.camera.single_video_screen import SingleVideoScreen


class ScreenController(QObject):
    def __init__(self, screenWidget=SingleVideoScreen(), topic="", imageType=CompressedImage, parent=None):
        super(ScreenController, self).__init__(parent)
        self.widget = screenWidget
        self.widget.playTopic.connect(self.change_topic)

        self.registered_topic = topic
        self.last_image = QImage()
        self.image_type = imageType
        self.subscriber = None
        self.subscribe()

    def image_callback(self, image):
        if self.image_type == CompressedImage:
            self.last_image = QImage.fromData(image.data)
        elif self.image_type == Image:
            #todo: convert uncompressed image to QImage
            pass

        self.widget.newSample(self.last_image)

    @pyqtSlot(str)
    def change_topic(self, topic):
        rospy.loginfo("Unregister $1, change to $2".format(self.registered_topic, topic))

        if self.subscriber is not None:
            self.subscriber.unregister()

        self.registered_topic = topic
        self.subscribe()

    def subscribe(self):
        if self.registered_topic is not "":
            self.subscriber = rospy.Subscriber(self.registered_topic, self.image_type, self.image_callback)

    def screenshot(self, filename):
        if not self.last_image.save(filename):
            rospy.logwarn("Image save unsuccessfull, topic: %1, file: %2".format(self.registered_topic, filename))

    def set_type(self, type):
        if type == Image:
            self.image_type = Image
        else:
            self.image_type =  CompressedImage