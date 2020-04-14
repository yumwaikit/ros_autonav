#!/usr/bin/env python

import math
import rospy
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

def halt():
    twist.linear = Vector3(0, 0, 0)
    twist.angular = Vector3(0, 0, 0)

def move_forward():
    twist.linear = Vector3(0.1, 0, 0)
    twist.angular = Vector3(0, 0, 0)

def turn_left():
    twist.linear = Vector3(0, 0, 0)
    twist.angular = Vector3(0, 0, math.pi / 2)

def turn_right():
    twist.linear = Vector3(0, 0, 0)
    twist.angular = Vector3(0, 0, -math.pi / 2)

def analyze_scan(laser_scan):
    rmin = laser_scan.range_max
    for i in len(laser_scan.ranges):
        if (i >= 0 and i < 10) or (i > len(laser_scan) - 10 and i < len(laser_scan)):
            r = laser_scan.ranges[i]
            if r > laser_scan.range_min and r < laser_scan.range_max and r < rmin:
                rmin = r
    if rmin < 0.3:
        halt()
    else
        move_forward()

def autorun_node():
    publisher = rospy.Publisher('cmd_vel', Twist, queue_size = 10)
    rospy.init_node('autorun_node')
    r = rospy.Rate(10)

    while not rospy.is_shutdown():
        rospy.init_node('sensor_node')
        rospy.Subscriber('scan', LaserScan, analyze_scan)
        rospy.loginfo(twist)
        publisher.publish(twist)
    	r.sleep()

if __name__=="__main__":
    try:
        twist = Twist()
        autorun_node()
    except rospy.ROSInterruptException:
        pass
