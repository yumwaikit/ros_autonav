#!/usr/bin/env python

import math
import rospy
from nav_msgs.msg import Odometry

def callback(od):
	#rospy.loginfo(od)
	angle = math.acos(od.pose.pose.orientation.w) * 2
	if od.pose.pose.orientation.z < 0:
		angle = -angle
	print('vector = ' + str(od.pose.pose.orientation.z))
	print('angle = ' + str(angle / math.pi * 180))

def receive():
	rospy.init_node('odometry_node')
	rospy.Subscriber('odom', Odometry, callback)
	rospy.spin()

if __name__=="__main__":
	try:
		receive()
	except rospy.ROSInterruptException:
		pass
