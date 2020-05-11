#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry

def callback(od):
    rospy.loginfo(od)

def receive():
    rospy.init_node('odometry_node')
    rospy.Subscriber('odom', Odometry, callback)
    rospy.spin()

if __name__=="__main__":
    try:
        receive()
    except rospy.ROSInterruptException:
        pass
