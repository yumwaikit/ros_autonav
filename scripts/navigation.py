#!/usr/bin/env python

import math
import numpy
import rospy
from std_msgs.msg import Bool
from std_msgs.msg import Int32
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from rospy.numpy_msg import numpy_msg

class Navigation:
	def __init__(self, track_device):
		while not rospy.is_shutdown():
			try:
				self.track_device = track_device

				if self.track_device == 1 or self.track_device == 3:
					odometry = rospy.wait_for_message('odom', Odometry, 0.5)
					self.init_odo_x = odometry.pose.pose.position.x
					self.init_odo_y = odometry.pose.pose.position.y
					self.init_odo_w = math.acos(odometry.pose.pose.orientation.w) * 2
					if odometry.pose.pose.orientation.z < 0:
						self.init_odo_w = -self.init_odo_w
					print("Odometry initial position: (" + str(self.init_odo_x) + ", " + str(self.init_odo_y) + ")")
					print("Odometry initial orientation: " + str(self.init_odo_w / math.pi * 180))
				if self.track_device == 2 or self.track_device == 3:
					rs_pose = rospy.wait_for_message('rs_pose', Float64MultiArray, 0.5)
					self.init_rs_x = rs_pose.data[0]
					self.init_rs_y = rs_pose.data[1]
					self.init_rs_w = math.acos(rs_pose.data[3]) * 2
					if rs_pose.data[2] < 0:
						self.init_rs_w = -self.init_rs_w
					print("Realsense initial position: (" + str(self.init_rs_x) + ", " + str(self.init_rs_y) + ")")
					print("Realsense initial orientation: " + str(self.init_rs_w / math.pi * 180))

				self.cur_x = 0
				self.cur_y = 0
				self.cur_w = 0
				self.target_x = 0
				self.target_y = 0
				self.target_w = 0
				self.turning = 0
				self.evading = 0
				print("Auto navigation Start")
				break
			except rospy.exceptions.ROSException:
				rospy.loginfo('Receive info timeout')
				
	def absolute_coordinates(self, x, y, track_device):
		if track_device == 1:
			init_x = self.init_odo_x
			init_y = self.init_odo_y
			w = self.init_odo_w
		elif track_device == 2:
			init_x = self.init_rs_x
			init_y = self.init_rs_y
			w = self.init_rs_w
		new_x = x * math.cos(w) - y * math.sin(w)
		new_y = x * math.sin(w) + y * math.cos(w)
		new_x = new_x + init_x
		new_y = new_y + init_y
		return new_x, new_y
	
	def relative_coordinates(self, x, y, track_device):
		if track_device == 1:
			init_x = self.init_odo_x
			init_y = self.init_odo_y
			w = self.init_odo_w
		elif track_device == 2:
			init_x = self.init_rs_x
			init_y = self.init_rs_y
			w = self.init_rs_w
		new_x = x - init_x
		new_y = y - init_y
		new_x = new_x * math.cos(-w) - y * math.sin(-w)
		new_y = new_x * math.sin(-w) + y * math.cos(-w)
		return new_x, new_y
				
	def set_target(self, target_x, target_y, target_w = 0):
		self.target_x = target_x
		self.target_y = target_y
		self.target_w = target_w

	def check_obstacle(self, laser_scan, distance, index, arc_length):
		dmin = laser_scan.range_max
		for i in range(len(laser_scan.ranges)):
			rotated = (i - index + len(laser_scan.ranges)) % len(laser_scan.ranges)
			if (rotated >= 0 and rotated < arc_length) or (rotated > len(laser_scan.ranges) - arc_length and rotated < len(laser_scan.ranges)):
				d = laser_scan.ranges[i]
				if d > laser_scan.range_min and d < laser_scan.range_max and d < dmin:
					dmin = d

		if dmin < distance:
			return True
		else:
			return False

	def find_closest_gap(self, laser_scan, distance, arc_length):
		ok_l = 0
		ok_r = 0
		for i in range(len(laser_scan.ranges) / 2):
			l = laser_scan.ranges[i]
			r = laser_scan.ranges[len(laser_scan.ranges) - i - 1]

			if l > laser_scan.range_min and l < laser_scan.range_max:
				if l >= distance:
					ok_l = ok_l + 1
				else:
					ok_l = 0
			if r > laser_scan.range_min and r < laser_scan.range_max:
				if r >= distance:
					ok_r = ok_r + 1
				else:
					ok_r = 0

			if ok_l >= arc_length:
				self.turning = -1
				return 2 #turn_left
			if ok_r >= arc_length:
				self.turning = 1
				return 3 #turn_right

		print('No gap available')
		return 4 #turn_around

	def navigate(self, laser_scan, rs_scan, status):
		dy = self.target_y - self.cur_y
		dx = self.target_x - self.cur_x
		print('remaining: (' + str(dy) + ', ' + str(dx) + ')')

		if status == 1 and abs(dy) < 0.5 and abs(dx) < 0.5:
			return 9 #continue
		elif status == 2 and abs(dy) < 0.05 and abs(dx) < 0.05:
			angle = self.target_w - self.cur_w
			if angle < -math.pi:
				angle = angle + math.pi * 2
			elif angle > math.pi:
				angle = angle - math.pi * 2
			print('angle: ' + str(angle / math.pi * 180))
			if angle > 0.05:
				return 2 #turn_left
			elif angle < -0.05:
				return 3 #turn_right
			else:
				return 10 #halt and break
		else:
			angle = math.atan2(dy, dx) - self.cur_w
			if angle < -math.pi:
				angle = angle + math.pi * 2
			elif angle > math.pi:
				angle = angle - math.pi * 2
			print('angle: ' + str(angle / math.pi * 180))

			if self.check_obstacle(laser_scan, 0.3, 0, 30):
				if self.evading > 0 and self.turning == -1:
					return 2 #turn_left
				elif self.evading > 0 and self.turning == 1:
					return 3 #turn_right
				else:
					return self.find_closest_gap(laser_scan, 0.4, 30)
			elif rs_scan == True:
				self.evading = 10
				if self.turning == -1:
					return 2 #turn_left
				else:
					return 3 #turn_right
			elif self.evading == 0 and angle < -0.05 and angle > -math.pi / 3 and not self.check_obstacle(laser_scan, 0.3, len(laser_scan.ranges) * 3 / 4, 90):
				self.turning = 1
				return 5 #advance_right
			elif self.evading == 0 and angle > 0.05 and angle < math.pi / 3 and not self.check_obstacle(laser_scan, 0.3, len(laser_scan.ranges) / 4, 90):
				self.turning = -1
				return 4 #advance_left
			elif self.evading == 0 and angle < -math.pi / 3 and not self.check_obstacle(laser_scan, 0.3, len(laser_scan.ranges) * 3 / 4, 90):
				self.turning = 1
				return 3 #turn_right
			elif self.evading == 0 and angle > math.pi / 3 and not self.check_obstacle(laser_scan, 0.3, len(laser_scan.ranges) / 4, 90):
				self.turning = -1
				return 2 #turn_left
			else:
				if self.evading > 0:
					self.evading = self.evading - 1
				self.turning = 0
				return 1 #forward

	def wait_next(self, status):
		try:
			laser_scan = rospy.wait_for_message('scan', LaserScan, 0.5)
			
			if self.track_device == 1:
				odometry = rospy.wait_for_message('odom', Odometry, 0.5)
				self.cur_x, self.cur_y = self.relative_coordinates(odometry.pose.pose.position.x, odometry.pose.pose.position.y, 1)
				self.cur_w = math.acos(odometry.pose.pose.orientation.w) * 2
				if odometry.pose.pose.orientation.z < 0:
					self.cur_w = -self.cur_w
				self.cur_w = self.cur_w - self.init_odo_w
				if status == 0:
					print('Odometry: (' + str(odometry.pose.pose.position.x) + ', ' + str(odometry.pose.pose.position.y) + ', ' + str(odometry.pose.pose.orientation.z) + ', ' + str(odometry.pose.pose.orientation.w) + ')')
			elif self.track_device == 2:
				rs_pose = rospy.wait_for_message('rs_pose', Float64MultiArray, 0.5)
				self.cur_x, self.cur_y = self.relative_coordinates(rs_pose.data[0], rs_pose.data[1], 2)
				self.cur_w = math.acos(rs_pose.data[3]) * 2
				if rs_pose.data[2] < 0:
					self.cur_w = -self.cur_w
				self.cur_w = self.cur_w - self.init_rs_w
				if status == 0:
					print('Odometry: (' + str(rs_pose.data[0]) + ', ' + str(rs_pose.data[1]) + ', ' + str(rs_pose.data[2]) + ', ' + str(rs_pose.data[3]) + ')')
			elif self.track_device == 3:
				odometry = rospy.wait_for_message('odom', Odometry, 0.5)
				rs_pose = rospy.wait_for_message('rs_pose', Float64MultiArray, 0.5)
				odo_x, odo_y = self.relative_coordinates(odometry.pose.pose.position.x, odometry.pose.pose.position.y, 1)
				rs_x, rs_y = self.relative_coordinates(rs_pose.data[0], rs_pose.data[1], 2)
				odo_w = math.acos(odometry.pose.pose.orientation.w) * 2
				if odometry.pose.pose.orientation.z < 0:
					odo_w = -odo_w
				odo_w = odo_w - self.init_odo_w
				rs_w = math.acos(rs_pose.data[3]) * 2
				if rs_pose.data[2] < 0:
					rs_w = -rs_w
				rs_w = rs_w - self.init_rs_w
				self.cur_x = (odo_x + rs_x) / 2
				self.cur_y = (odo_y + rs_y) / 2
				self.cur_w = (odo_w + rs_w + math.pi * 8) / 2 - math.pi * 4
				
			if self.cur_w < -math.pi:
				self.cur_w = self.cur_w + math.pi * 2
			elif self.cur_w > math.pi:
				self.cur_w = self.cur_w - math.pi * 2
				
			print('Current position: (' + str(self.cur_x) + ', ' + str(self.cur_y) + ')')
			print('Current orientation: ' + str(self.cur_w / math.pi * 180))
			
			rs_scan = rospy.wait_for_message('rs_depth', Bool, 0.5)
			#rs_frame = rospy.wait_for_message('rs_color', numpy_msg(Int32), 10)
			
			if status > 0:
				return self.navigate(laser_scan, rs_scan.data, status)
			else:
				return 0
		except rospy.exceptions.ROSException:
			rospy.loginfo('Receive info timeout')
			return 0
