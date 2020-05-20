#!/usr/bin/env python

import math
import numpy
import rospy
from std_msgs.msg import Bool
from std_msgs.msg import Int32
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from rospy.numpy_msg import numpy_msg

class Navigation:
	def __init__(self, relative):
		while not rospy.is_shutdown():
			try:
				self.relative = relative
				odometry = rospy.wait_for_message('odom', Odometry, 0.5)

				self.init_x = odometry.pose.pose.position.x
				self.init_y = odometry.pose.pose.position.y
				self.init_w = math.acos(odometry.pose.pose.orientation.w) * 2
				if odometry.pose.pose.orientation.z < 0:
					self.init_w = -self.init_w

				print("Auto navigation Start")
				print("Initial position: (" + str(self.init_x) + ", " + str(self.init_y) + ")")
				print("Initial angle: " + str(self.init_w / math.pi * 180))

				self.cur_x = self.init_x
				self.cur_y = self.init_y
				self.target_x = self.init_x
				self.target_y = self.init_y
				self.target_w = self.init_w
				self.evading = 0
				break
			except rospy.exceptions.ROSException:
				rospy.loginfo('Receive info timeout')
				
	def set_target(self, target_x, target_y, target_w = 0):
		if self.relative:
			self.target_x = target_x * math.cos(self.init_w) - target_y * math.sin(self.init_w)
			self.target_y = target_x * math.sin(self.init_w) + target_y * math.cos(self.init_w)
			self.target_x = self.target_x + self.init_x
			self.target_y = self.target_y + self.init_y
			self.target_w = self.target_w + target_w
			if self.target_w < -math.pi:
				self.target_w = self.target_w + math.pi * 2
			elif self.target_w > math.pi:
				self.target_w = self.target_w - math.pi * 2
		else:
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
				return 2 #turn_left
			if ok_r >= arc_length:
				return 3 #turn_right

		print('No gap available')
		return 4 #turn_around

	def navigate(self, laser_scan, pose, rs_scan, is_last):
		dy = self.target_y - pose.position.y
		dx = self.target_x - pose.position.x
		angle = math.acos(pose.orientation.w) * 2
		if pose.orientation.z < 0:
			angle = -angle
		print('remaining: (' + str(dy) + ', ' + str(dx) + ')')

		if (not is_last) and abs(dy) < 0.5 and abs(dx) < 0.5:
			return 9 #continue
		elif is_last and abs(dy) < 0.05 and abs(dx) < 0.05:
			angle = self.target_w - angle
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
			angle = math.atan2(dy, dx) - angle
			if angle < -math.pi:
				angle = angle + math.pi * 2
			elif angle > math.pi:
				angle = angle - math.pi * 2
			print('angle: ' + str(angle / math.pi * 180))

			if self.check_obstacle(laser_scan, 0.3, 0, 30):
				return self.find_closest_gap(laser_scan, 0.4, 30)
			elif rs_scan == True:
				self.evading = 10
				return 3 #turn_right
			elif self.evading == 0 and angle < -0.05 and angle > -math.pi / 3 and not self.check_obstacle(laser_scan, 0.3, len(laser_scan.ranges) * 3 / 4, 90):
				return 5 #advance_right
			elif self.evading == 0 and angle > 0.05 and angle < math.pi / 3 and not self.check_obstacle(laser_scan, 0.3, len(laser_scan.ranges) / 4, 90):
				return 4 #advance_left
			elif self.evading == 0 and angle < -math.pi / 3 and not self.check_obstacle(laser_scan, 0.3, len(laser_scan.ranges) * 3 / 4, 90):
				return 3 #turn_right
			elif self.evading == 0 and angle > math.pi / 3 and not self.check_obstacle(laser_scan, 0.3, len(laser_scan.ranges) / 4, 90):
				return 2 #turn_left
			else:
				if self.evading > 0:
					self.evading = self.evading - 1
				return 1 #forward

	def wait_next(self, is_last):
		try:
			laser_scan = rospy.wait_for_message('scan', LaserScan, 0.5)
			odometry = rospy.wait_for_message('odom', Odometry, 0.5)
			rs_scan = rospy.wait_for_message('rs_depth', Bool, 0.5)
			#rs_frame = rospy.wait_for_message('rs_color', numpy_msg(Int32), 10)
			#rospy.loginfo(rs_frame)
			
			if self.relative:
				cur_x = odometry.pose.pose.position.x - self.init_x
				cur_y = odometry.pose.pose.position.y - self.init_y
				self.cur_x = cur_x * math.cos(-self.init_w) - cur_y * math.sin(-self.init_w)
				self.cur_y = cur_x * math.sin(-self.init_w) + cur_y * math.cos(-self.init_w)
			else:
				self.cur_x = odometry.pose.pose.position.x
				self.cur_y = odometry.pose.pose.position.y
			print('Current location: (' + str(self.cur_x) + ', ' + str(self.cur_y) + ')')

			return self.navigate(laser_scan, odometry.pose.pose, rs_scan.data, is_last)
		except rospy.exceptions.ROSException:
			rospy.loginfo('Receive info timeout')
			return 0
