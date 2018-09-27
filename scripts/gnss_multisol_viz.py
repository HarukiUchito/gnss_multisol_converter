#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from trajectory_viz_python.srv import AddNewPoint
from latlon_to_xy_service.srv import ConvLatLon
from rtklibros.msg import *
import message_filters

def add_marker(name, x, y):
    rospy.wait_for_service('trajectory_server')
    try:
        add_new_point = rospy.ServiceProxy('trajectory_server', AddNewPoint)
        resp1 = add_new_point(name, x, y)
        return resp1
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def conv_latlon(lat, lon):
    rospy.wait_for_service('latlon_to_xy_service')
    try:
        convLatLon = rospy.ServiceProxy('latlon_to_xy_service', ConvLatLon)
        resp1 = convLatLon(lat, lon)
        return resp1
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

ini_lat = rospy.get_param("/gnss_ini_lat")
ini_lon = rospy.get_param("/gnss_ini_lon")
iniXY = conv_latlon(ini_lat, ini_lon)

calcedOnce = False
sub_all = PoseWithCovarianceStamped()
sub_msg = rtklibros.msg.solutions()

ms_msg = message_filters.Subscriber('multi_solutions', solutions)
ms_all = message_filters.Subscriber('latlon', PoseWithCovarianceStamped)

def callback(msg, msg_all):
    global sub_msg, sub_all, calcedOnce
    sub_msg = msg
    sub_all = msg_all
    calcedOnce = True

def main():
    global sub_msg, sub_all, iniX, iniY, calcedOnce
    rospy.init_node('gnss_multisol_viz', anonymous=True)
    ts = message_filters.ApproximateTimeSynchronizer([ms_msg, ms_all], 1, 1)
    ts.registerCallback(callback)
    
    r = rospy.Rate(1)
    while not rospy.is_shutdown():
        if calcedOnce:
            solAll = conv_latlon(sub_all.pose.pose.position.x, sub_all.pose.pose.position.y)
            add_marker("ALL", solAll.y - iniXY.y, solAll.x - iniXY.x)
        for sol in sub_msg.solutions:
            solXY = conv_latlon(sol.latitude, sol.longitude)
            add_marker(sol.excludedSatellite, solXY.y - iniXY.y, solXY.x - iniXY.x)
        r.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException: pass
