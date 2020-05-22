#!/usr/bin/env python

import math
import rospy
from nav_msgs.msg import Odometry

class Diagnose:
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

				print("Diagnose Start")
				print("Initial position: (" + str(self.init_x) + ", " + str(self.init_y) + ")")
				print("Initial angle: " + str(self.init_w / math.pi * 180))

				self.cur_x = self.init_x
				self.cur_y = self.init_y
				self.cur_w = self.init_w
				break
			except rospy.exceptions.ROSException:
				rospy.loginfo('Receive info timeout')
				
	def wait_next(self):
		try:
			odometry = rospy.wait_for_message('odom', Odometry, 0.5)
			
			if self.relative:
				cur_x = odometry.pose.pose.position.x - self.init_x
				cur_y = odometry.pose.pose.position.y - self.init_y
				self.cur_x = cur_x * math.cos(-self.init_w) - cur_y * math.sin(-self.init_w)
				self.cur_y = cur_x * math.sin(-self.init_w) + cur_y * math.cos(-self.init_w)
				cur_w = math.acos(odometry.pose.pose.orientation.w) * 2
				if odometry.pose.pose.orientation.z < 0:
					cur_w = -cur_w
				self.cur_w = cur_w - self.init_w
				if self.cur_w < -math.pi:
					self.cur_w = self.cur_w + math.pi * 2
				elif self.cur_w > math.pi:
					self.cur_w = self.cur_w - math.pi * 2
			else:
				self.cur_x = odometry.pose.pose.position.x
				self.cur_y = odometry.pose.pose.position.y
				self.cur_w = math.acos(pose.orientation.w) * 2
				if odometry.pose.pose.orientation.z < 0:
					self.cur_w = -self.cur_w

			print('Current position: (' + str(self.cur_x) + ', ' + str(self.cur_y) + ')')
			print('Current orientation: ' + str(self.cur_w / math.pi * 180))

		except rospy.exceptions.ROSException:
			rospy.loginfo('Receive info timeout')