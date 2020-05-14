#!/usr/bin/env python

import math
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Odometry
from movements import Movement
from manipulations import Joints
from manipulations import Gripper
from navigation import Navigation

move = None
joints = None
gripper = None
nav = None
r = None

def pick_up_item():
	global joints
	global gripper
	joints.pick()
	rospy.sleep(1)
	gripper.grab()
	rospy.sleep(1)
	joints.hold()

def drop_item():
	global joints
	global gripper
	joints.pick()
	rospy.sleep(1)
	gripper.release()
	rospy.sleep(1)
	joints.neutral()

def turn_around():
	global move
	global r
	try:
		odometry = rospy.wait_for_message('odom', Odometry, 0.5)
		init_angle = math.acos(odometry.pose.pose.orientation.w) * 2
		if odometry.pose.pose.orientation.z < 0:
			init_angle = -init_angle
		cur_angle = init_angle
		while abs(cur_angle + init_angle) > 0.05 and not rospy.is_shutdown():
			move.turn_left()
			r.sleep()
			try:
				odometry = rospy.wait_for_message('odom', Odometry, 0.5)
				cur_angle = math.acos(odometry.pose.pose.orientation.w) * 2
				if odometry.pose.pose.orientation.z < 0:
					cur_angle = -cur_angle
			except rospy.exceptions.ROSException:
				print('Receive info timeout')
				print('Halt')
				move.halt()
	except rospy.exceptions.ROSException:
		print('Receive info timeout')
		print('Halt')

def navigate(target_x, target_y):
	global move
	global nav
	global r

	nav = Navigation()

	while not rospy.is_shutdown():
		output = nav.wait_next(target_x, target_y, True)
		if output == 0:
			print('Halt')
			move.halt()
		elif output == 1:
			print('Forward')
			move.forward()
		elif output == 2:
			print('Turn left')
			move.turn_left()
		elif output == 3:
			print('Turn right')
			move.turn_right()
		elif output == 4:
			print('Turn around')
			turn_around()
		elif output == 10:
			print('Exit')
			move.halt()
			break
		r.sleep()

def autorun_node():
	global move
	global joints
	global gripper
	global nav
	global r
	
	move_publisher = rospy.Publisher('cmd_vel', Twist, queue_size = 5)
	joints_publisher = rospy.Publisher('joint_trajectory_point', Float64MultiArray, queue_size = 5)
	gripper_publisher = rospy.Publisher('gripper_position', Float64MultiArray, queue_size = 5)
	move = Movement(move_publisher)
	joints = Joints(joints_publisher)
	gripper = Gripper(gripper_publisher)

	rospy.init_node('autorun_node')
	r = rospy.Rate(10)
	navigate(0.5, 0.0)

if __name__=="__main__":
	try:
		print('Start autorun')
		autorun_node()
	except rospy.ROSInterruptException:
		pass
