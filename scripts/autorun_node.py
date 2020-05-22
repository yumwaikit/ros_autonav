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
from diagnostic_info import Diagnose
from select_data import DataSelect
from shortest_path import Dijkstra

move = None
joints = None
gripper = None
nav = None
diag = None
data = None
dij = None
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

def navigate(is_last):
	global move
	global gripper
	global nav
	global r

	while not rospy.is_shutdown():
		output = nav.wait_next(is_last)
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
			print('Advance left')
			move.advance_left()
		elif output == 5:
			print('Advance right')
			move.advance_right()
		elif output == 6:
			print('Turn around')
			turn_around()
		elif output == 9:
			print('Arrive at node, continue')
			break;
		elif output == 10:
			print('Arrive')
			move.halt()
			gripper.grab()
			rospy.sleep(1)
			gripper.release()
			print('Exit Auto navigation')
			break
		r.sleep()

def navigate_to_point(target_x, target_y):
	global nav
	global data
	global dij
	
	nodes = data.select_nodes()
	start = -1
	end = -1
	d_min = float('inf')
	for node in nodes:
		d = math.sqrt((nodes[node][1] - nav.cur_y) ** 2 + (nodes[node][0] - nav.cur_x) ** 2)
		print(str(node) + ': ' + str(d))
		if d < d_min:
			d_min = d
			start = node
	d_min = float('inf')
	for node in nodes:
		d = math.sqrt((nodes[node][1] - target_y) ** 2 + (nodes[node][0] - target_x) ** 2)
		if d < d_min:
			d_min = d
			end = node
	
	path = dij.get_path(start, end)
	points = map(lambda node: nodes[node], path)
	for point in points:
		nav.set_target(point[0], point[1])
		navigate(False)
	nav.set_target(target_x, target_y)
	navigate(True)

def autorun_node():
	global move
	global joints
	global gripper
	global nav
	global diag
	global data
	global dij
	global r
	
	move_publisher = rospy.Publisher('cmd_vel', Twist, queue_size = 5)
	joints_publisher = rospy.Publisher('joint_trajectory_point', Float64MultiArray, queue_size = 5)
	gripper_publisher = rospy.Publisher('gripper_position', Float64MultiArray, queue_size = 5)

	rospy.init_node('autorun_node')
	r = rospy.Rate(10)
	
	move = Movement(move_publisher)
	joints = Joints(joints_publisher)
	gripper = Gripper(gripper_publisher)
	data = DataSelect()
	dij = Dijkstra(data.select_graph())
	
	d = raw_input('Press Enter to start calibration.')
	if d == 'd':
		diag = Diagnose(True)
		joints.neutral()
		gripper.release()
		print('Calibration complete.')
		print('=========================================')
		print('ROS Auto-Navigation Node')
		print('---------Diagnostic mode')
		print('\n')
		try:
			while not rospy.is_shutdown():
				diag.wait_next()
				rospy.sleep(1)
		except rospy.ROSInterruptException:
			print('Node terminated')
			pass
	else:
		nav = Navigation(True)
		joints.neutral()
		gripper.release()
		print('Calibration complete.')
		try:
			while not rospy.is_shutdown():
				print('=========================================')
				print('ROS Auto-Navigation Node')
				print('\n')
				print('Please input the target coordinates.')
				x = float(raw_input('x = '))
				y = float(raw_input('y = '))
				navigate_to_point(x, y)
		except ValueError:
			print('Node terminated')
			pass

if __name__=="__main__":
	try:
		autorun_node()
	except rospy.ROSInterruptException:
		pass
