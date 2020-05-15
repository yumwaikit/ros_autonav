#!/usr/bin/env python

import math
import rospy
from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from rospy.numpy_msg import numpy_msg

class Navigation:
	def __init__(self):
		while True:
			try:
				odometry = rospy.wait_for_message('odom', Odometry, 0.5)

				self.offset_x = odometry.pose.pose.position.x
				self.offset_y = odometry.pose.pose.position.y
				self.offset_w = math.acos(odometry.pose.pose.orientation.w) * 2
				if odometry.pose.pose.orientation.z < 0:
					self.offset_w = -self.offset_w

				self.target_x = 0
				self.target_y = 0
				self.evading = 0
				break
			except rospy.exceptions.ROSException:
				rospy.loginfo('Receive info timeout')

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
				return 2 #turn_left
			if ok_r >= arc_length:
				return 3 #turn_right

		print('No gap available')
		return 4 #turn_around

	def navigate(self, laser_scan, pose, rs_scan):
		dy = self.target_y - pose.position.y
		dx = self.target_x - pose.position.x
		print('remaining: (' + str(dy) + ', ' + str(dx) + ')')

		if abs(dy) < 0.05 and abs(dx) < 0.05:
			return 10 #halt and break
		else:
			angle = math.acos(pose.orientation.w) * 2
			if pose.orientation.z < 0:
				angle = -angle
			angle = math.atan2(dy, dx) - angle
			if angle < -math.pi:
				angle = angle + math.pi * 2
			elif angle > math.pi:
				angle = angle - math.pi * 2
			print('angle: ' + str(angle))

			if self.check_obstacle(laser_scan, 0.3, 0, 30):
				self.evading = 10
				return self.find_closest_gap(laser_scan, 0.4, 30)
			elif rs_scan == True:
				self.evading = 10
				return 2 #turn_left
			elif self.evading == 0 and angle < -0.05 and not self.check_obstacle(laser_scan, 0.3, len(laser_scan.ranges) * 3 / 4, 90):
				return 3 #turn_right
			elif self.evading == 0 and angle > 0.05 and not self.check_obstacle(laser_scan, 0.3, len(laser_scan.ranges) / 4, 90):
				return 2 #turn_left
			else:
				if self.evading > 0:
					print('Evade motion')
					self.evading = self.evading - 1
				return 1 #forward

	def wait_next(self, target_x, target_y, relative):
		try:
			laser_scan = rospy.wait_for_message('scan', LaserScan, 0.5)
			odometry = rospy.wait_for_message('odom', Odometry, 0.5)
			rs_scan = rospy.wait_for_message('rs_depth', Bool, 0.5)
			rs_frame = rospy.wait_for_message('rs_color', numpy_msg(Int32), 0.5)

			if relative:
				self.target_x = target_x * math.cos(self.offset_w) - target_y * math.sin(self.offset_w)
				self.target_y = target_x * math.sin(self.offset_w) + target_y * math.cos(self.offset_w)
				self.target_x = self.target_x + self.offset_x
				self.target_y = self.target_y + self.offset_y
			else:
				self.target_x = target_x
				self.target_y = target_y

			return self.navigate(laser_scan, odometry.pose.pose, rs_scan.data)
		except rospy.exceptions.ROSException:
			rospy.loginfo('Receive info timeout')
			return 0
