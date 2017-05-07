#!/usr/bin/env python

import cv2
import argparse

if __name__ == '__main__':

    FOV = 80  # Field of view of mast cam (degrees)

    # construct the argument parse and parse the arguments
    ap = argparse.ArgumentParser()
    ap.add_argument("-p", "--panorama", required=True, help="path to the panoramic image")
    ap.add_argument("-c", "--coordinate", required=True, help="path to the center coordinate of the first (left) frame")
    ap.add_argument("-f", "--frame_width", required=True, help="width (pixels) of a single frame in the panorama")
    args = vars(ap.parse_args())

    pan = cv2.imread(args["panorama"])
    coord = int(args["coordinate"])
    width = args["frame_width"]

    ratio = float(width)/FOV  # Number of pixels per degrees
    print ratio

    # Start location of scale
    startPos = float(width)/2 - ratio*(float(coord) % 5)
    startAngle = coord - (coord % 5)

    for i in range(0, 40):
        markerPos = startPos+ratio*(i*5)
        markerAngle = startAngle + i*5

        if markerPos > pan.shape[1]:
            break

        if markerAngle >= 360:
            markerAngle = markerAngle - 360

        if markerAngle == 0:
            cv2.line(pan, (int(markerPos), 50), (int(markerPos), 75), (255, 255, 255), 6)
            cv2.putText(pan, "N", (int(markerPos)-10, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 3)
            cv2.putText(pan, str(markerAngle), (int(markerPos)-7, 105), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 255, 255), 2)
        elif markerAngle == 90:
            cv2.line(pan, (int(markerPos), 50), (int(markerPos), 75), (255, 255, 255), 6)
            cv2.putText(pan, "E", (int(markerPos)-10, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 3)
            cv2.putText(pan, str(markerAngle), (int(markerPos)-13, 105), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 255, 255), 2)
        elif markerAngle == 180:
            cv2.line(pan, (int(markerPos), 50), (int(markerPos), 75), (255, 255, 255), 6)
            cv2.putText(pan, "S", (int(markerPos)-10, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 3)
            cv2.putText(pan, str(markerAngle), (int(markerPos)-13, 105), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 255, 255), 2)
        elif markerAngle == 270:
            cv2.line(pan, (int(markerPos), 50), (int(markerPos), 75), (255, 255, 255), 6)
            cv2.putText(pan, "W", (int(markerPos)-10, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 3)
            cv2.putText(pan, str(markerAngle), (int(markerPos)-13, 105), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 255, 255), 2)
        else:
            cv2.line(pan, (int(markerPos), 55), (int(markerPos), 70), (255, 255, 255), 2)
            cv2.putText(pan, str(markerAngle), (int(markerPos)-10, 95), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

    for i in range(0, 40):
        markerPos = startPos - ratio*(i*5)
        markerAngle = startAngle - i*5

        if markerPos < 0:
            break

        if markerAngle < 0:
            markerAngle = markerAngle + 360

        if markerAngle == 0:
            cv2.line(pan, (int(markerPos), 50), (int(markerPos), 75), (255, 255, 255), 6)
            cv2.putText(pan, "N", (int(markerPos)-10, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 3)
            cv2.putText(pan, str(markerAngle), (int(markerPos)-7, 105), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 255, 255), 2)
        elif markerAngle == 90:
            cv2.line(pan, (int(markerPos), 50), (int(markerPos), 75), (255, 255, 255), 6)
            cv2.putText(pan, "E", (int(markerPos)-10, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 3)
            cv2.putText(pan, str(markerAngle), (int(markerPos)-13, 105), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 255, 255), 2)
        elif markerAngle == 180:
            cv2.line(pan, (int(markerPos), 50), (int(markerPos), 75), (255, 255, 255), 6)
            cv2.putText(pan, "S", (int(markerPos)-10, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 3)
            cv2.putText(pan, str(markerAngle), (int(markerPos)-13, 105), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 255, 255), 2)
        elif markerAngle == 270:
            cv2.line(pan, (int(markerPos), 50), (int(markerPos), 75), (255, 255, 255), 6)
            cv2.putText(pan, "W", (int(markerPos)-10, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 3)
            cv2.putText(pan, str(markerAngle), (int(markerPos)-13, 105), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 255, 255), 2)
        else:
            cv2.line(pan, (int(markerPos), 55), (int(markerPos), 70), (255, 255, 255), 2)
            cv2.putText(pan, str(markerAngle), (int(markerPos)-10, 95), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

    # show the images
    cv2.imshow("Panorama", pan)

    cv2.waitKey(0)
