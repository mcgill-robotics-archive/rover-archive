"""!@brief Controller for a single screen."""
import rospy
from PyQt5.QtCore import QObject, pyqtSlot
from PyQt5.QtGui import QImage
from sensor_msgs.msg import CompressedImage, Image


class ScreenController(QObject):
    """!@brief Controller for a single screen.

    Manages the subscriber and the change of topics for each screen.
    Handles the recording of a screenshot for the frame
    """

    def __init__(self, screen_widget=None, topic="", image_type=CompressedImage, parent=None):
        """!@brief Constructor registers the associated widget and subscriber

        @param self Python object pointer
        @param screen_widget Pointer to SingleVideoScreen object to be managed
        by this controller
        @param image_type The ros message type of the image to receive
        @param parent The Qt parent
        """
        super(ScreenController, self).__init__(parent)
        self.widget = screen_widget
        self.widget.playTopic.connect(self.change_topic)

        self.registered_topic = topic
        self.last_image = QImage()
        self.image_type = image_type
        self.subscriber = None
        self._subscribe()

    def image_callback(self, image):
        """!@brief Ros subscriber callback

         Converts the ros image message to a qimage then forwards it to the
         display widget

        @param self Python object pointer
        @param image The ros message

        """
        if self.image_type == CompressedImage:
            self.last_image = QImage.fromData(image.data)
        elif self.image_type == Image:
            if image.encoding == "mono8":
                image_type = QImage.Format_Indexed8
            else:
                image_type = QImage.Format_RGB888
            self.last_image = QImage(image.data, image.width, image.height, image_type)

        self.widget.new_sample(self.last_image)

    @pyqtSlot(str)
    def change_topic(self, topic):
        """!@brief Slot for the topic selection combo box

        Changes the registered topic for the image subscriber

        @param self Python object pointer
        @param topic The new topic name string
        """
        rospy.loginfo("Unregister {0}, change to {1}".format(self.registered_topic, topic))

        if self.subscriber is not None:
            self.subscriber.unregister()

        self.registered_topic = topic
        self._subscribe()

    def _subscribe(self):
        if self.registered_topic is not None and self.registered_topic is not "":
            self.subscriber = rospy.Subscriber(self.registered_topic, self.image_type, self.image_callback)

    def screenshot(self, filename):
        """!@brief Save current frame to disk

        @param filename The name of the file to save to
        @param self Python object pointer
        """
        if not self.last_image.save(filename):
            rospy.logwarn("Image save unsuccessful, topic: {0}, file: {1}".format(self.registered_topic, filename))

    def set_type(self, image_type):
        if image_type == Image:
            self.image_type = Image
        else:
            self.image_type = CompressedImage
