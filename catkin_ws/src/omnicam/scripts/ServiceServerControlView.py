#!/usr/bin/env python

import rospy
from omnicam.srv import *

position_x = 0
position_y = 0
position_zoom = 0

def receive_view_command(request):
    global position_zoom
    global position_x
    global position_y
    position_zoom += request.zoom
    position_y += request.y
    position_x += request.x
    print (position_x,position_y,position_zoom)
    return ControlViewResponse(True)

rospy.init_node("crop_node_contol_test")
service = rospy.Service("crop_control", ControlView, receive_view_command)
rospy.spin()


