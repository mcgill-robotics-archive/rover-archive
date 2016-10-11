import rospy
import rosgraph
import math

import sys

if sys.version_info >= (3,0):
    deg_symb = chr(176)
else:
    deg_symb = unichr(176)

## Get the hostname of the ROS master
#
# @return the ros master hostname
#
def parse_master_uri():
    uri = rosgraph.get_master_uri()
    uri = uri[7:]
    uri = uri[:uri.find(':')]
    rospy.loginfo("Master is URI:" + uri)
    return uri


## Change the background of an object to red 
#
# @param thing An object implementing the QWidget Interface. This function uses
# the setStyleSheet function of the interface.
def lbl_bg_red(thing):
    """sets a style sheet to the @param thing resulting in a red background"""
    thing.setStyleSheet('background-color:#ff0000')
    thing.setText("Bad")


## Change the background of an object to the default light gray background color 
#
# @param thing An object implementing the QWidget Interface. This function uses
# the setStyleSheet function of the interface.
def lbl_bg_norm(thing):
    """sets a style sheet to the @param thing resulting in a green background"""
    thing.setStyleSheet('background-color:#33CC33')
    thing.setText("Ok")


## Change the background of an object to red
#
# @param thing An object implementing the QWidget Interface. This function uses
# the setStyleSheet function of the interface.
def lbl_bg_red(thing, text):
    """sets a style sheet to the @param thing resulting in a red background"""
    thing.setStyleSheet('background-color:#ff0000')
    thing.setText(text)


## Change the background of an object to the default light gray background color
#
# @param thing An object implementing the QWidget Interface. This function uses
# the setStyleSheet function of the interface.
def lbl_bg_grn(thing, text):
    """sets a style sheet to the @param thing resulting in a green background"""
    thing.setStyleSheet('background-color:#33CC33')
    thing.setText(text)


## Change angle representation from Decimal to DMS format
#
# @param dec_deg The decimal representation of the number to convert.
# 
# @return The same angle in DMS representation
#
def format_dms(dec_deg):
    deg = int(dec_deg)
    mins = (abs(dec_deg - deg) * 60)
    mins_i = int(mins)
    sec = (mins - mins_i) * 60

    return "%i%c %i' %.3f''" % (deg, deg_symb, mins_i, sec)

## Convert a radian angle to degrees
#
# @param angle An angle in radians
#
# @return The same angle in degrees with 2 decimals
#
def format_euler_angle(angle):
    deg = math.degrees(angle)
    string = "%.2f" % deg
    return string + deg_symb
