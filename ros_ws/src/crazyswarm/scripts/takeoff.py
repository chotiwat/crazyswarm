#!/usr/bin/env python

import sys
import rospy
from crazyflie_driver.srv import *

def takeoff_client(group, height, time_from_start):
    rospy.wait_for_service('takeoff')
    try:
        takeoff = rospy.ServiceProxy('/takeoff', Takeoff)
        resp1 = takeoff(group, height, rospy.Duration(time_from_start))
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def usage():
    return "%s [group height(m) time_from_start(s)]"%sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 4:
        group = int(sys.argv[1])
        height = float(sys.argv[2])
        time_from_start = float(sys.argv[3])
    else:
        print usage()
        sys.exit(1)
    print "Requesting takeoff group=%s height=%s time_from_start=%s"%(group, height, time_from_start)
    takeoff_client(group, height, time_from_start)