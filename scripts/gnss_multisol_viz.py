#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import pyproj
from geometry_msgs.msg import PoseWithCovarianceStamped
from trajectory_viz_python.srv import AddNewPoint
import message_filters
from rtklibros.msg import *

EPSG4612 = pyproj.Proj("+init=EPSG:4612")
EPSG2451 = pyproj.Proj("+init=EPSG:2451")

def add_marker(name, x, y):
    rospy.wait_for_service('trajectory_server')
    try:
        add_new_point = rospy.ServiceProxy('trajectory_server', AddNewPoint)
        resp1 = add_new_point(name, x, y)
        return resp1
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

ini_y,ini_x = 0.0, 0.0
sub_msg = solutions()

ms_msg = message_filters.Subscriber('multi_solutions', solutions)
ms_ini_xy = message_filters.Subscriber('gnss_ini_xy', PoseWithCovarianceStamped)

def callback(msg, msg_ini_xy):
    print msg
    print msg_ini_xy
    ini_x = msg_ini_xy.pose.pose.position.x
    ini_y = msg_ini_xy.pose.pose.position.y
        
    for sol in msg.solutions:
        y,x = pyproj.transform(EPSG4612, EPSG2451, sol.longitude,sol.latitude)
        add_marker(sol.excludedSatellite, y - ini_y, x - ini_x)   
    

def main():
    rospy.init_node('gnss_multisol_viz', anonymous=True)
    ts = message_filters.ApproximateTimeSynchronizer([ms_msg, ms_ini_xy], 1, 1)
    ts.registerCallback(callback)

    r = rospy.Rate(1)
    while not rospy.is_shutdown():
        r.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException: pass
