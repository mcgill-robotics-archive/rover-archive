#!/usr/bin/env python

import json
import cv2

class UnwarpConfigGenerator(object):
    def __init__(self, filename):
        
        self.vals = []
        self.setCount = 0
        self.cap = cv2.VideoCapture(0)
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

    #mouse callback function
    def setCoord(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDBLCLK:
            print "Callback"
            coord = (x,y)
            self.vals.append(coord)
            self.setCount += 1
        if self.setCount == 3:
            self.write_to_file(self.filename)
            print ("Parameters written in " + self.filename)
            self.quit()

    def quit(self):
        self.cap.release()
        cv2.destroyAllWindows()
        exit(0)

    def write_to_file(self, filename):
        f = open(filename, 'w')
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
    test = UnwarpConfigGenerator("test.json")
