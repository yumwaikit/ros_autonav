#!/usr/bin/env python

import math
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Empty
from nav_msgs.msg import Odometry
from movements import Movement
from manipulations import Joints
from manipulations import Gripper
from navigation import Navigation
from select_data import DataSelect
from shortest_path import Dijkstra
from autorun_gui import Controller

class AutorunNode:
	def __init__(self):
		self.reset_publisher = rospy.Publisher('reset', Empty, queue_size = 5)
		move_publisher = rospy.Publisher('cmd_vel', Twist, queue_size = 5)
		joints_publisher = rospy.Publisher('joint_trajectory_point', Float64MultiArray, queue_size = 5)
		gripper_publisher = rospy.Publisher('gripper_position', Float64MultiArray, queue_size = 5)

		rospy.init_node('autorun_node')
		self.r = rospy.Rate(10)
	
		self.move = Movement(move_publisher)
		self.joints = Joints(joints_publisher)
		self.gripper = Gripper(gripper_publisher)
		self.data = DataSelect()
		self.dij = Dijkstra(self.data.select_graph())
		self.nav = None
		self.win = None
		
		self.continue_run = False
		
	def execute(self):
		arg = raw_input('Press Enter to start calibration.')
		self.win = Controller('OfficeMap.jpg', self.data.select_graph())
		self.reset_publisher.publish(Empty())
		self.joints.neutral()
		self.gripper.release()
		self.nav = Navigation(True)
		print('Calibration complete.')
		if arg == 'd':
			print('=========================================')
			print('ROS Auto-Navigation Node')
			print('---------Diagnostic mode')
			print('\n')
			try:
				while not rospy.is_shutdown():
					self.nav.wait_next(0)
					self.win.update(self.nav.cur_x, self.nav.cur_y, self.nav.cur_w)
					rospy.sleep(1)
			except rospy.ROSInterruptException:
				print('Node terminated')
				pass
		elif arg == 'c':
			try:
				while not rospy.is_shutdown():
					print('=========================================')
					print('ROS Auto-Navigation Node')
					print('\n')
					coordinates = raw_input('Please input the target coordinates (x,y)...')
					if coordinates.count(',') == 0:
						print('Node terminated')
						break
					elif coordinates.count(',') <> 1:
						print('Not a proper coordinate string.')
						continue
					else:
						xy = coordinates.split(',')
						try:
							x = float(xy[0])
							y = float(xy[1])
							self.gripper.grab()
							self.navigate_to_point(x, y)
							self.continue_run = False
						except ValueError:
							print('Not a proper coordinate string.')
							continue
			except rospy.ROSInterruptException:
				print('Node terminated')
				pass
		else:
			self.win.canvas.bind("<Button-1>", self.window_clicked)
			try:
				while not rospy.is_shutdown():
					print('=========================================')
					print('ROS Auto-Navigation Node')
					print('\n')
					raw_input('Please click target location on map...')
			except rospy.ROSInterruptException:
				print('Node terminated')
				pass				
	
	def window_clicked(self, event):
		x, y = self.win.reverse_transform(event.x, event.y)
		self.continue_run = False
		self.win.display_destination(event.x, event.y)
		rospy.sleep(1)
		self.gripper.grab()
		self.navigate_to_point(x, y)
		if self.continue_run:
			print('=========================================')
			print('ROS Auto-Navigation Node')
			print('\n')
			raw_input('Please click target location on map...')
			self.continue_run = False
			
	def navigate_to_point(self, target_x, target_y):
		nodes = self.data.select_nodes()
		start = -1
		end = -1
		d_min = float('inf')
		for node in nodes:
			d = math.sqrt((nodes[node][1] - self.nav.cur_y) ** 2 + (nodes[node][0] - self.nav.cur_x) ** 2)
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
	
		path = self.dij.get_path(start, end)
		self.continue_run = True
		points = map(lambda node: nodes[node], path)
		for point in points:
			if self.continue_run:
				self.nav.set_target(point[0], point[1])
				self.navigate(1)
			else:
				break
		if self.continue_run:
			self.nav.set_target(target_x, target_y)
			self.navigate(2)
		
	def navigate(self, status):
		while not rospy.is_shutdown() and self.continue_run:
			output = self.nav.wait_next(status)
			self.win.update(self.nav.cur_x, self.nav.cur_y, self.nav.cur_w)
			if output == 0:
				print('Halt')
				self.move.halt()
			elif output == 1:
				print('Forward')
				self.move.forward()
			elif output == 2:
				print('Turn left')
				self.move.turn_left()
			elif output == 3:
				print('Turn right')
				self.move.turn_right()
			elif output == 4:
				print('Advance left')
				self.move.advance_left()
			elif output == 5:
				print('Advance right')
				self.move.advance_right()
			elif output == 6:
				print('Turn around')
				self.turn_around()
			elif output == 9:
				print('Arrive at node, continue')
				break;
			elif output == 10:
				print('Arrive')
				self.move.halt()
				self.gripper.grab()
				rospy.sleep(1)
				self.gripper.release()
				print('Exit Auto navigation')
				break
			self.r.sleep()	

	def turn_around(self):
		try:
			odometry = rospy.wait_for_message('odom', Odometry, 0.5)
			init_angle = math.acos(odometry.pose.pose.orientation.w) * 2
			if odometry.pose.pose.orientation.z < 0:
				init_angle = -init_angle
			cur_angle = init_angle
			while abs(cur_angle + init_angle) > 0.05 and not rospy.is_shutdown():
				self.move.turn_left()
				self.r.sleep()
				try:
					odometry = rospy.wait_for_message('odom', Odometry, 0.5)
					cur_angle = math.acos(odometry.pose.pose.orientation.w) * 2
					if odometry.pose.pose.orientation.z < 0:
						cur_angle = -cur_angle
				except rospy.exceptions.ROSException:
					print('Receive info timeout')
					print('Halt')
					self.move.halt()
		except rospy.exceptions.ROSException:
			print('Receive info timeout')
			print('Halt')
			
	def pick_up_item(self):
		self.joints.pick()
		rospy.sleep(1)
		self.gripper.grab()
		rospy.sleep(1)
		self.joints.hold()

	def drop_item(self):
		self.joints.pick()
		rospy.sleep(1)
		self.gripper.release()
		rospy.sleep(1)
		self.joints.neutral()