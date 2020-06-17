#!/usr/bin/env python

import math
import rospy
import threading
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Empty
from movements import Movement
from manipulations import Joints
from manipulations import Gripper
from navigation import Navigation
from select_data import DataSelect
from shortest_path import Dijkstra
from autorun_gui import Controller


class AutorunNode:
	def __init__(self):
		rospy.init_node('autorun_node')
		self.r = rospy.Rate(10)

		self.reset_publisher = {}
		self.rs_reset_publisher = {}
		self.move = {}
		self.joints = {}
		self.gripper = {}
		self.nav = {}
		self.continue_run = {}
		self.threads = {}
		self.win = None
		self.data = DataSelect()
		self.robot_ids = self.data.select_robots()
		self.dij = Dijkstra(self.data.select_graph())
		self.track_device = 2
		self.mode = -1
		
		for i in self.robot_ids.keys():
			self.reset_publisher[i] = rospy.Publisher('/' + str(i) + '/reset', Empty, queue_size=5)
			self.rs_reset_publisher[i] = rospy.Publisher('/' + str(i) + '/rs_reset', Empty, queue_size=5)

			move_publisher = rospy.Publisher('/' + str(i) + '/cmd_vel', Twist, queue_size=5)
			self.move[i] = Movement(move_publisher)
			joints_publisher = rospy.Publisher('/' + str(i) + '/joint_trajectory_point', Float64MultiArray, queue_size=5)
			self.joints[i] = Joints(joints_publisher)
			gripper_publisher = rospy.Publisher('/' + str(i) + '/gripper_position', Float64MultiArray, queue_size=5)
			self.gripper[i] = Gripper(gripper_publisher)
			self.continue_run[i] = False

	def execute(self):
		while not rospy.is_shutdown():
			arg = raw_input('Press Enter to start calibration.')
			self.win = Controller('OfficeMap.jpg', self.data.select_graph(), self.robot_ids)

			for i in self.robot_ids.keys():
				self.threads[i] = threading.Thread(target=self.init_robot, args=[i])
				self.threads[i].start()
			for i in self.threads.keys():
				self.threads[i].join()
			for i in self.robot_ids.keys():
				self.win.update(i, self.nav[i].cur_x, self.nav[i].cur_y, self.nav[i].cur_w)

			self.win.window.update_idletasks()
			self.win.window.update()

			print('Calibration complete.')
			if arg == 'd':
				self.mode = 0
				self.diagnose_mode()
			elif arg == 'c':
				self.mode = 1
				self.command_mode()
			else:
				self.mode = 2
				self.gui_mode()
			break

	def init_robot(self, i):
		self.reset_publisher[i].publish(Empty())
		self.rs_reset_publisher[i].publish(Empty())
		self.move[i].halt()
		self.joints[i].neutral()
		self.gripper[i].release()
		self.nav[i] = Navigation(i, self.robot_ids[i][0], self.robot_ids[i][1], self.track_device)
			
	def diagnose_mode(self):
		print('=========================================')
		print('ROS Auto-Navigation Node')
		print('---------Diagnostic mode')
		print('\n')
		try:
			while not rospy.is_shutdown():
				nav = self.nav[self.robot_ids.keys()[0]]
				nav.wait_next(0)
				self.win.update(self.robot_ids.keys()[0], nav.cur_x + nav.offset_x, nav.cur_y + nav.offset_y, nav.cur_w)
				self.r.sleep()
		except rospy.ROSInterruptException:
			print('Node terminated')
			pass
		
	def command_mode(self):
		try:
			while not rospy.is_shutdown():
				print('=========================================')
				print('ROS Auto-Navigation Node')
				print('\n')
				coordinates = raw_input('Please input the target coordinates (x,y)...')
				if coordinates.count(',') == 0:
					print('Node terminated')
					break
				elif coordinates.count(',') != 1:
					print('Not a proper coordinate string.')
					continue
				else:
					xy = coordinates.split(',')
					try:
						x = float(xy[0])
						y = float(xy[1])
						self.gripper[self.robot_ids.keys()[0]].grab()
						self.navigate_to_point(self.robot_ids.keys()[0], x, y)
						self.continue_run = False
					except ValueError:
						print('Not a proper coordinate string.')
						continue
		except rospy.ROSInterruptException:
			print('Node terminated')
			pass
		
	def gui_mode(self):
		self.win.canvas.bind("<Button-1>", lambda event, arg=1: self.window_clicked(event, arg))
		self.win.canvas.bind("<Button-3>", lambda event, arg=2: self.window_clicked(event, arg))
		try:
			while not rospy.is_shutdown():
				for i in self.robot_ids.keys():
					self.win.update(i, self.nav[i].cur_x + self.nav[i].offset_x, self.nav[i].cur_y + self.nav[i].offset_y, self.nav[i].cur_w)
				# print('=========================================')
				# print('ROS Auto-Navigation Node')
				# print('\n')
				# print('Click target location on map to navigate...')
				# raw_input('Press Enter to quit.')
				# print('Node terminated')
				# break
		except rospy.ROSInterruptException:
			print('Node terminated')
			pass
	
	def window_clicked(self, event, i):
		x, y = self.win.reverse_transform(event.x, event.y)
		self.win.display_destination(i, event.x, event.y)
		self.continue_run[i] = False
		self.threads[i].join()
		self.gripper[i].grab()
		self.continue_run[i] = True
		self.threads[i] = threading.Thread(target=self.navigate_to_point, args=(i, x, y))
		self.threads[i].start()
			
	def navigate_to_point(self, i, target_x, target_y):
		nodes = self.data.select_nodes()
		start = -1
		end = -1
		d_min = float('inf')
		for node in nodes:
			d = math.sqrt((nodes[node][1] - self.nav[i].cur_y) ** 2 + (nodes[node][0] - self.nav[i].cur_x) ** 2)
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
		self.continue_run[i] = True
		points = map(lambda n: nodes[n], path)
		for point in points:
			if self.continue_run[i]:
				self.nav[i].set_target(point[0], point[1])
				self.navigate(i, 1)
			else:
				break
		if self.continue_run[i]:
			self.nav[i].set_target(target_x, target_y)
			self.navigate(i, 2)
		self.continue_run[i] = False
		
	def navigate(self, i, status):
		while self.continue_run[i] and not rospy.is_shutdown():
			output = self.nav[i].wait_next(status)
			if self.mode != 2:
				self.win.update(i, self.nav[i].cur_x + self.nav[i].offset_x, self.nav[i].cur_y + self.nav[i].offset_y, self.nav[i].cur_w)
			if output == 0:
				print('Halt')
				self.move[i].halt()
			elif output == 1:
				print('Forward')
				self.move[i].forward()
			elif output == 2:
				print('Turn left')
				self.move[i].turn_left()
			elif output == 3:
				print('Turn right')
				self.move[i].turn_right()
			elif output == 4:
				print('Advance left')
				self.move[i].advance_left()
			elif output == 5:
				print('Advance right')
				self.move[i].advance_right()
			elif output == 6:
				print('Turn right')
				self.move[i].turn_right()
			elif output == 9:
				print('Arrive at node, continue')
				break
			elif output == 10:
				print('Arrive')
				self.move[i].halt()
				self.gripper[i].grab()
				rospy.sleep(1)
				self.gripper[i].release()
				print('Exit Auto navigation')
				break
			self.r.sleep()
			
	def pick_up_item(self, i):
		self.joints[i].pick()
		rospy.sleep(1)
		self.gripper[i].grab()
		rospy.sleep(1)
		self.joints[i].hold()

	def drop_item(self, i):
		self.joints[i].pick()
		rospy.sleep(1)
		self.gripper[i].release()
		rospy.sleep(1)
		self.joints[i].neutral()
