#!/usr/bin/env python
import sys
import rospy
from auto_nav.srv import *

def reset_client(reset_input):
    rospy.wait_for_service('reset')
    try:
        reset = rospy.ServiceProxy('reset', Reset)
        didReset = reset(reset_input)
        return Reset.didReset
    except rospy.ServiceException, e:
        print "Service call failed %s"%e

def usage():
    return "%s 'rs'"%sys.argv[0]

if __name__ == "__main__":
    if(len(sys.argv) == 2 && sys.argv[1] == "rs"):
        rs = "rs"
    else:
        print usage()
        sys.exit(1)
    print "Trying to reset"
    didReset = reset_client(rs)
    if(didReset):
        print "Reset successful!"
    else:
        print "Reset failed!"
