#!/usr/bin/python
import rospy
import time
from sensor_msgs.msg import CompressedImage
import sys


class ReduceFPS():
    def __init__(self):

        self.temp_image = None
        self.last_image = None
        self.pub = rospy.Publisher("/reduced/compressed", CompressedImage, queue_size=10)
        rospy.Subscriber("/image_mono/compressed", CompressedImage, self.receive_image)

        self.fps = float(rospy.get_param('~fps', 15))
        print self.fps

        while not rospy.is_shutdown():
            self.publish_image()
            time.sleep(1/self.fps)

    def receive_image(self, image):
        self.temp_image = image

    def publish_image(self):
        if self.temp_image is not None:
            if self.temp_image is not self.last_image:
                self.pub.publish(self.temp_image)
                self.last_image = self.temp_image

# Main function.
if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('fpsReduce')
    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        fps = ReduceFPS()
    except rospy.ROSInterruptException:
        pass
