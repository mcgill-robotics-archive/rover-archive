#!/usr/bin/env python

import json
import cv2

## Generate the configuration parameters for the Unwarper class
#
class UnwarpConfigGenerator(object):
    
    ## Class constructor
    #
    # Connects to the camera and starts the mouse event capture.
    #
    # @param filename The file where the config will be saved.
    # @param video_device The camera to use for calibration. Can use device 
    # index (int) or terminal device name (/dev/video0)
    #
    def __init__(self, filename, video_device):
        
        self.vals = []
        self.setCount = 0
        self.cap = cv2.VideoCapture(video_device)
        self.filename = filename

        #Set capture resolution
        self.cap.set(3,1280)
        self.cap.set(4,1024)

        while True:
            ret, frame = self.cap.read()
            cv2.imshow('SetupFeed', frame)
            cv2.setMouseCallback('SetupFeed', self.setCoord)

            if cv2.waitKey(1) == ord('q'):
                break

    ## Mouse event callback function
    #
    # Is called whenever a mouse event is registered by the framework
    #
    def setCoord(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDBLCLK:
            print "Callback"
            coord = (x,y)
            self.vals.append(coord)
            self.setCount += 1
        if self.setCount == 3:
            self.write_to_file()
            print ("Parameters written in " + self.filename)
            self.quit()

    ## Terminate the process
    #
    def quit(self):
        self.cap.release()
        cv2.destroyAllWindows()
        exit(0)

    ## Write the generated parameters to file for the main script
    # 
    def write_to_file(self):
        f = open(self.filename, 'w')
        try:
            dictionary = {
                "center":self.vals[0], 
                "inner":self.vals[1], 
                "outer": self.vals[2]
            }
            json.dump(dictionary, f, indent=4)
        except IndexError as e:
            print e

        f.close()

if __name__ == '__main__':
    test = UnwarpConfigGenerator("test.json", "/dev/omnicam")
