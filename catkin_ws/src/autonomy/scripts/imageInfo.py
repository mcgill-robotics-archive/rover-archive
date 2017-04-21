# Get position and BGR color information of a clicked feature

import cv2
import numpy as np

x_center, y_center = 0,0
trackedBGR = [0,0,0]
trackedHSV = [0,0,0]

minimumArea = 10
maximumArea = 0
x,y,w,h = 0,0,0,0
bestContour = None
currentArea = 0


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
    global x_center, y_center, trackedHSV, trackedBGR

    # Reduce noise, median blur was better so far
    #blur = cv2.blur(frame,(5,5))
    #blur = cv2.GaussianBlur(frame,(5,5),5)
    blur = cv2.medianBlur(image,5)

    trackedBGR = blur[y_center,x_center]
    print "\nBGR values: B: %r   G: %r   R: %r " %(trackedBGR[0],trackedBGR[1],trackedBGR[2])

    trackedHSV = BGRtoHSV(trackedBGR)

    print "\nHSV values: H: %r   S: %r   V: %r " %(trackedHSV[0],trackedHSV[1],trackedHSV[2])

def BGRtoHSV(bgr_array):

    h,s,v = 0,0,0

    Bp = bgr_array[0]/255.0
    Gp = bgr_array[1]/255.0
    Rp = bgr_array[2]/255.0

    Cmax = float(max(Bp, Gp, Rp))
    Cmin = float(min(Bp, Gp, Rp))
    delta = float(Cmax-Cmin)

    if delta == 0.0:
        h = 0.0
    elif Cmax == Rp:
        h = 60*((Gp-Bp)/delta)
    elif Cmax == Gp:
        h = 60*(((Bp-Rp)/delta)+2)
    else:
        h = 60*(((Rp-Gp)/delta)+4)

    if Cmax == 0.0:
        s = 0.0
    else:
        s = delta/Cmax

    v = Cmax

    # OPENCV works with values of H going from 0 to 180 (not 0 to 360),
    # S and V range from 0 to 255
    return [abs(h/2),s*255,v*255]


cv2.namedWindow('WebcamFeed')
cv2.setMouseCallback('WebcamFeed',mouseCallBack)

while(1):
    ret, frame = cap.read()

    lower_bound = np.array([trackedHSV[0]-10,trackedHSV[1]-50,trackedHSV[2]-70])
    upper_bound = np.array([trackedHSV[0]+10,trackedHSV[1]+50,trackedHSV[2]+70])

    #blur_img = cv2.medianBlur(frame,5)
    blur_img = cv2.GaussianBlur(frame,(5,5),5)

    hsv_img = cv2.cvtColor(blur_img, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv_img, lower_bound, upper_bound)

    #Get rid of background noise using erosion and fill in the holes using dilation
    element = cv2.getStructuringElement(cv2.MORPH_RECT,(5,5))
    eroded = cv2.erode(mask,element, iterations=1)
    dilated = cv2.dilate(eroded,element,iterations=6)


    # Identify the right feature on the dilated image
    img3, contours, hierarchy = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    for contour in contours:
        currentArea = cv2.contourArea(contour)
        x,y,w,h = cv2.boundingRect(contour)

        #Compare position with the current display to avoid jumps
        if bestContour is not None:
            x_base, y_base, w_base, h_base = cv2.boundingRect(bestContour) 

        #add if condition
        if currentArea > maximumArea:
            bestContour = contour
            maximumArea = currentArea

    #Create a bounding box around the biggest object in color range
    if bestContour is not None:
        
        if currentArea > minimumArea:
            itemDetected = True
        
        x,y,w,h = cv2.boundingRect(bestContour)
        cv2.rectangle(frame, (x,y),(x+w,y+h), (0,0,255), 3)
    font = cv2.FONT_HERSHEY_SIMPLEX

    cv2.putText(frame,'Position (%d,%d)' %(x,y),(10,50), font, 1,(255,255,255),2)
    print 'Area: ', currentArea

    cv2.imshow('WebcamFeed', frame)
    cv2.imshow('mask', mask)
    cv2.imshow('Eroded', eroded)
    cv2.imshow('Dilated', dilated)
        
    k = cv2.waitKey(20) & 0xFF
    if k == ord('q'):
        break
    elif k == ord('a'):
        print ix,iy

cv2.destroyAllWindows()
