# Get position and BGR color information of a clicked feature

import cv2
import numpy as np

x_center, y_center = 0,0
trackedBGR = [0,0,0]


cap = cv2.VideoCapture(0)

# mouse callback function
def mouseCallBack(event,x,y,flags,param):
    global x_center, y_center, trackedHSV
    if event == cv2.EVENT_LBUTTONDBLCLK:
        print "Clicked @   X = %r   Y = %r" %(x, y)
        x_center = x
        y_center = y

        getColor(frame)


def getColor(image):
    global x_center, y_center, trackedHSV

    # Reduce noise, median blur was better so far
    #blur = cv2.blur(frame,(5,5))
    #blur = cv2.GaussianBlur(frame,(5,5),5)
    blur = cv2.medianBlur(image,5)

    trackedBGR = blur[y_center,x_center]
    print "\nBGR values: B: %r   G: %r   R: %r " %(trackedBGR[0],trackedBGR[1],trackedBGR[2])

cv2.namedWindow('WebcamFeed')
cv2.setMouseCallback('WebcamFeed',mouseCallBack)

while(1):
    ret, frame = cap.read()

    cv2.imshow('WebcamFeed', frame)
        
    k = cv2.waitKey(20) & 0xFF
    if k == ord('q'):
        break
    elif k == ord('a'):
        print ix,iy

cv2.destroyAllWindows()
