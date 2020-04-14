#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import LaserScan

def callback(mf):
    rospy.loginfo(mf)

def receive():
    rospy.init_node('sensor_node')
    rospy.Subscriber('scan', LaserScan, callback)
    rospy.spin()

if __name__=="__main__":
    try:
        receive()
    except rospy.ROSInterruptException:
        pass
