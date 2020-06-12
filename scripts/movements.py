#!/usr/bin/env python

from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3


class Movement:
	def __init__(self, publisher):
		self.publisher = publisher
		self.twist = Twist()

	def halt(self):
		self.twist.linear = Vector3(0, 0, 0)
		self.twist.angular = Vector3(0, 0, 0)
		self.publisher.publish(self.twist)

	def forward(self):
		self.twist.linear = Vector3(0.1, 0, 0)
		self.twist.angular = Vector3(0, 0, 0)
		self.publisher.publish(self.twist)

	def turn_left(self):
		self.twist.linear = Vector3(0, 0, 0)
		self.twist.angular = Vector3(0, 0, 0.2)
		self.publisher.publish(self.twist)

	def turn_right(self):
		self.twist.linear = Vector3(0, 0, 0)
		self.twist.angular = Vector3(0, 0, -0.2)
		self.publisher.publish(self.twist)
		
	def advance_left(self):
		self.twist.linear = Vector3(0.1, 0, 0)
		self.twist.angular = Vector3(0, 0, 0.2)
		self.publisher.publish(self.twist)	

	def advance_right(self):
		self.twist.linear = Vector3(0.1, 0, 0)
		self.twist.angular = Vector3(0, 0, -0.2)
		self.publisher.publish(self.twist)
