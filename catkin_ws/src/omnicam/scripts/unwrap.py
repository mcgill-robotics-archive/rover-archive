#!/usr/bin/env python

import json
import numpy as np
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

## Takes a spherical image and unwraps it to a flat image
#
class Unwraper(object):

    ## Constructor uses configuration structure to deternime image parameters
    #
    # @param config Configuration object of type UnwraperConfiguration
    # @param bw Use or not the black and white image from the camera
    #
    def __init__(self, config, bw=True):
        self.config = config
        self.map_x = None
        self.map_y = None

        if bw:
            self.encoding = "mono8"
            self.publisher_topic = "unwarp_GRAY"
            self.subscriber_topic = "/omnicam/image_mono"
        else:
            self.encoding = "bgr8"
            self.publisher_topic = "unwarp_BGR"
            self.subscriber_topic = "/omnicam/image_color"

        rospy.init_node("omnicam_unwarp", anonymous=False)
        self.publisher = rospy.Publisher(self.publisher_topic, Image, queue_size=1)

        self.build_map()
        self.bridge = CvBridge()

        self.subscriber = rospy.Subscriber(self.subscriber_topic, Image, self.callback, queue_size=1)
        rospy.loginfo("Unwrap loaded")
        rospy.spin()

    ##Initialise a proper sized image to be filled with the unwarped image
    #
    def build_map(self):
        self.map_x = np.zeros((self.config.Hd, self.config.Wd), np.float32)
        self.map_y = np.zeros((self.config.Hd, self.config.Wd), np.float32)

        for y in xrange(0, int(self.config.Hd - 1)):
            for x in xrange(0,int(self.config.Wd - 1)):
                r = (float(y) / float(self.config.Hd)) * (self.config.R2 - self.config.R1) + self.config.R1
                theta = (float(x) / float(self.config.Wd)) * 2.0 * np.pi
                xS = self.config.Cx + r * np.sin(theta)
                yS = self.config.Cy + r * np.cos(theta)
                self.map_x.itemset((y,x),int(xS))
                self.map_y.itemset((y,x),int(yS))
        
    ## Actual function to unwarp the image.
    #
    # Uses the open cv library to unwarp the image with the internal map.
    #
    # @param img The spherical image to be unwarped
    def unwarp(self, img):
        output = cv2.remap(img, self.map_x, self.map_y, cv2.INTER_LINEAR)
        return output

    ## ROS callback function
    #
    # Callback function called by the subscriber when receiving a new image.
    # The new image is unwraped upon arrival.
    #
    # @param data The newly arrived ros image.
    def callback(self, data):
        try:
            cv_incoming = self.bridge.imgmsg_to_cv2(data, self.encoding)
            result = self.unwarp(cv_incoming)
            self.publisher.publish(self.bridge.cv2_to_imgmsg(result, self.encoding))
        except CvBridgeError as e:
            print(e)


## Read and aglomerate all the configuration parameters relevant to the unwarping algorithm
#
# The constructor will attempt to read from file all the values for the parameters.
#
class UnwraperConfiguration(object):

    ## Class constructor
    #
    # @param filepath File where to read the configuration from. It should be 
    # json formated with 3 coordinate pairs labeled 'center', 'inner', 'outer'.
    #
    def __init__(self, filepath):
        self.filepath = filepath
        
        # center of the "donut"    
        self.Cx = 0
        self.Cy = 0
        # Inner donut radius
        self.R1x = 0
        self.R1y = 0
        # outer donut radius
        self.R2x = 0
        self.R2y = 0

        with open(self.filepath) as data_file:
            data = json.load(data_file)
 
        # center of the "donut"    
        self.Cx = data["center"][0]
        self.Cy = data["center"][1]
        # Inner donut radius
        self.R1x = data["inner"][0]
        self.R1y = data["inner"][1]
        self.R1 = self.R1x - self.Cx
        # outer donut radius
        self.R2x = data["outer"][0]
        self.R2y = data["outer"][1]
        self.R2 = self.R2x - self.Cx
        
        # our input and output image siZes
        self.Wd = 2.0 * ((self.R2 + self.R1) / 2) * np.pi
        self.Hd = (self.R2 - self.R1)


if __name__ == '__main__':
    filename = rospy.get_param("config_file", "../config/omnicam_config.json")
    config = UnwraperConfiguration(filename)
    unwarp = Unwraper(config, False)

        
