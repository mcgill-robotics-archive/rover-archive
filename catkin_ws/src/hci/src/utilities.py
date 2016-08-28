import rospy
import rosgraph
import math

# TODO: implement watchdog reset with new architecture 
# def reset_watchdog():
#     # reset watchdog
#     rospy.wait_for_service("reset_watchdog", 0.1)
# 
#     try:
#         reset = rospy.ServiceProxy("reset_watchdog", ResetWatchDog)
#         response = reset(1, 10)
#         if response.Response != 555:
#             rospy.logerr("Bad response")
# 
#     except rospy.ServiceException, e:
#         print "Service call failed: %s", e

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

    return "%i%c %i' %.3f''" % (deg, chr(176), mins_i, sec)

## Convert a radian angle to degrees
#
# @param angle An angle in radians
#
# @return The same angle in degrees with 2 decimals
#
def format_euler_angle(angle):
    deg = math.degrees(angle)
    string = "%.2f" % deg
    return string
