#!/usr/bin/env python

import math
from std_msgs.msg import Float64MultiArray


class Joints:
	def __init__(self, publisher):
		self.publisher = publisher
		self.pos = Float64MultiArray()

	def neutral(self):
		self.pos.data = [1.0, 0.0, -math.pi / 16, math.pi * 7 / 16, -math.pi * 3 / 8]
		self.publisher.publish(self.pos)

	def hold(self):
		self.pos.data = [1.0, 0.0, -math.pi / 4, math.pi * 3 / 8, -math.pi / 4]
		self.publisher.publish(self.pos)

	def pick(self):
		self.pos.data = [1.0, 0.0, math.pi / 4, 0.0, math.pi / 4]
		self.publisher.publish(self.pos)

	def rotate(self, angle):
		self.pos.data[1] = angle
		self.publisher.publish(self.pos)


class Gripper:
	def __init__(self, publisher):
		self.publisher = publisher
		self.pos = Float64MultiArray()

	def grab(self):
		self.pos.data = [1.0]
		self.publisher.publish(self.pos)

	def release(self):
		self.pos.data = [0.0]
		self.publisher.publish(self.pos)
