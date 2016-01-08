#!/usr/bin/env python

import numpy as np
import cv2


cap = cv2.VideoCapture(0)

first_shot = True
x,y = 0,0
old_x, old_y, = 0,0


def readPos(current_x,current_y):
    
    return (current_x,current_y)
    

while(True):
    # Capture frame-by-frame.
    ret, frame = cap.read()
    # Our operations on the frame come here.
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    if (first_shot):
        x, old_x = frame.shape[1]/2.0, frame.shape[1]/2.0
        y, old_y = frame.shape[0]/2.0, frame.shape[0]/2.0

        w = frame.shape[1]
        h = 100.0

        first_shot = False
    
    x,y = readPos(old_x, old_y)
    w = h/480*640
    
    #Crop the feed ([startY:endY, startX:endX] format)
    crop_img = gray[y-h/2:y+h/2, x-w/2:x+w/2]
    
    
    #Resize the feed to a 480 x 640 format
    resized = cv2.resize(crop_img, (640, 480))

    
    # Display the resulting frames.
    cv2.imshow('Black & White Frame', gray)
    cv2.imshow('Cropped', crop_img)
    cv2.imshow('resized', resized)
    
    old_x = x
    old_y = y

    # Quit on holding ESC
    if cv2.waitKey(1) == 27:
        break
        

# When everything done, release the capture.
cap.release()
cv2.destroyAllWindows()
