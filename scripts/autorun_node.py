#!/usr/bin/env python

import math
import rospy
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

twist = None
joints = None
gripper = None
publisher = None
joint_publisher = None
gripper_publisher = None
direction = 0
continue_run = True
r = None

def halt():
    global twist
    global publisher
    global continue_run
    twist.linear = Vector3(0, 0, 0)
    twist.angular = Vector3(0, 0, 0)
    continue_run = False
    publisher.publish(twist)

def move_forward():
    global twist
    twist.linear = Vector3(0.1, 0, 0)
    twist.angular = Vector3(0, 0, 0)

def turn_left():
    global direction
    global twist
    direction = direction - 1
    twist.linear = Vector3(0, 0, 0)
    twist.angular = Vector3(0, 0, 0.4)

def turn_right():
    global direction
    global twist
    direction = direction + 1
    twist.linear = Vector3(0, 0, 0)
    twist.angular = Vector3(0, 0, -0.4)

def joints_neutral():
    global joints
    global joints_publisher
    joints.data = [1.0, 0.0, -math.pi * 3 / 8, math.pi / 4, math.pi / 8]
    joint_publisher.publish(joints)

def joints_hold():
    global joints
    global joint_publisher
    joints.data = [1.0, 0.0, -math.pi / 4, math.pi * 3 / 8, -math.pi / 4]
    joint_publisher.publish(joints)

def joints_pick():
    global joints
    global joint_publisher
    joints.data = [1.0, 0.0, math.pi / 4, 0.0, math.pi / 4]
    joint_publisher.publish(joints)

def joints_shake():
    global joints
    global joint_publisher
    joints.data = [1.0, 0.5, -math.pi * 3 / 8, math.pi / 4, math.pi / 8]
    joint_publisher.publish(joints)
    rospy.sleep(0.5)
    joints.data = [1.0, -0.5, -math.pi * 3 / 8, math.pi / 4, math.pi / 8]
    joint_publisher.publish(joints)
    rospy.sleep(0.5)
    joints_neutral()

def gripper_grip():
    global gripper
    global gripper_publisher
    gripper.data = [1.0]
    gripper_publisher.publish(gripper)

def gripper_release():
    global gripper
    global gripper_publisher
    gripper.data = [0.0]
    gripper_publisher.publish(gripper)

def check_distance(laser_scan, min_distance, index):
    rmin = laser_scan.range_max
    for i in range(len(laser_scan.ranges)):
        rotated = (i - index + len(laser_scan.ranges)) % len(laser_scan.ranges)
        if (rotated >= 0 and rotated < 20) or (rotated > len(laser_scan.ranges) - 20 and rotated < len(laser_scan.ranges)):
            r = laser_scan.ranges[i]
            if r > laser_scan.range_min and r < laser_scan.range_max and r < rmin:
                rmin = r
    if rmin < min_distance:
        return True
    else:
        return False

def analyze_scan(laser_scan):
    global twist
    global publisher
    if check_distance(laser_scan, 0.4, 0):
        turn_left()
    elif direction < 0 and not check_distance(laser_scan, 0.4, len(laser_scan.ranges) * 3 / 4):
        turn_right()
    elif direction > 0 and not check_distance(laser_scan, 0.4, len(laser_scan.ranges) / 4):
        turn_left()
    else:
        move_forward()
    if continue_run:
    	publisher.publish(twist)

def pick_up_item():
    joints_pick()
    rospy.sleep(1)
    gripper_grip()
    rospy.sleep(1)
    joints_hold()

def drop_item():
    joints_pick()
    rospy.sleep(1)
    gripper_release()
    rospy.sleep(1)
    joints_neutral()

def autorun_node(use_sensor = True):
    global twist
    global joints
    global gripper
    global publisher
    global joint_publisher
    global gripper_publisher
    global direction
    global continue_run
    global r
    
    publisher = rospy.Publisher('cmd_vel', Twist, queue_size = 10)
    joint_publisher = rospy.Publisher('joint_trajectory_point', Float64MultiArray, queue_size = 10)
    gripper_publisher = rospy.Publisher('gripper_position', Float64MultiArray, queue_size = 10)
    twist = Twist()
    joints = Float64MultiArray()
    gripper = Float64MultiArray()
    direction = 0
    continue_run = True

    rospy.init_node('autorun_node')
    r = rospy.Rate(10)

    for i in range(300):
        if use_sensor:
            try:
                laser_scan = rospy.wait_for_message('scan', LaserScan, 1)
                analyze_scan(laser_scan)
    	        r.sleep()
            except rospy.exceptions.ROSException:
                move_forward()
    	        publisher.publish(twist)
                rospy.sleep((300 - i) / 10)
                break
        else:
            move_forward()
    	    publisher.publish(twist)
            r.sleep()
    halt()
    rospy.sleep(1)
    joints_shake()
    rospy.sleep(1)
    pick_up_item()
    rospy.sleep(1)
    continue_run = True
    for i in range(45):
        turn_left()
        publisher.publish(twist)
        r.sleep()
    halt()
    rospy.sleep(1)
    direction = 0
    continue_run = True
    for i in range(300):
        if use_sensor:
            try:
                laser_scan = rospy.wait_for_message('scan', LaserScan, 1)
                analyze_scan(laser_scan)
    	        r.sleep()
            except rospy.exceptions.ROSException:
                move_forward()
    	        publisher.publish(twist)
                rospy.sleep((300 - i) / 10)
                break
        else:
            move_forward()
    	    publisher.publish(twist)
            r.sleep()
    halt()
    rospy.sleep(1)
    drop_item()

if __name__=="__main__":
    try:
        rospy.loginfo('start autorun')
        autorun_node(True)
    except rospy.ROSInterruptException:
        pass
