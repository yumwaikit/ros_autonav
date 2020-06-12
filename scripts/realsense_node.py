#!/usr/bin/env python

import os
import numpy
import rospy
import pyrealsense2 as rs
from std_msgs.msg import Empty
from std_msgs.msg import Bool
from std_msgs.msg import Float64MultiArray
from rospy.numpy_msg import numpy_msg


class RealsenseNode:
	def __init__(self):
		self.pose_publisher = rospy.Publisher('rs_pose', Float64MultiArray, queue_size=1)
		self.depth_publisher = rospy.Publisher('rs_depth', Bool, queue_size=1)
		#self.color_publisher = rospy.Publisher('rs_color', numpy_msg(Int32), queue_size = 1)
		rospy.init_node('realsense_node')
		
		self.ctx = rs.context()		
		self.pose_pipeline = self.create_pipeline(os.environ['T265_SERIAL'])
		self.depth_pipeline = self.create_pipeline(os.environ['D435_SERIAL'])
	
	def execute(self):
		try:
			while not rospy.is_shutdown():
				try:
					rospy.wait_for_message('rs_reset', Empty, 0.1)
					print('Reset pose tracking.')
					self.depth_pipeline.stop()
					self.pose_pipeline.stop()
					rospy.sleep(0.5)
					self.pose_pipeline = self.create_pipeline(os.environ['T265_SERIAL'])
					self.depth_pipeline = self.create_pipeline(os.environ['D435_SERIAL'])
				except rospy.exceptions.ROSException:
					if self.pose_pipeline:
						pose_data = self.grab_pose_frame()
						if pose_data:
							array = Float64MultiArray()
							array.data = [[-pose_data.translation.z, -pose_data.translation.x], [pose_data.rotation.y, pose_data.rotation.w]]
							self.pose_publisher.publish(array)
					if self.depth_pipeline:
						color, depth = self.grab_depth_frame()
						if depth:
							has_obstacle = self.check_ground_obstacle(numpy.asanyarray(depth.as_frame().get_data()))
							self.depth_publisher.publish(has_obstacle)
						#if color:
							#self.color_publisher.publish(numpy.asanyarray(color.as_frame().get_data()))
		except Exception:
			if self.pose_pipeline:
				self.pose_pipeline.stop()
			if self.depth_pipeline:
				self.depth_pipeline.stop()
			raise

	def create_pipeline(self, serial):
		pipe = rs.pipeline(self.ctx)
		cfg = rs.config()
		cfg.enable_device(serial)
		pipe.start(cfg)
		print('Realsense camera ' + serial + ' started.')
		return pipe
							
	def grab_pose_frame(self):
		frames = None
		pose_data = None
		if self.pose_pipeline:
			frames = self.pose_pipeline.wait_for_frames()
			pose = frames.get_pose_frame()
			if pose:
				pose_data = pose.get_pose_data()
		return pose_data
	
	def grab_depth_frame(self):
		frames = None
		color = None
		depth = None
		if self.depth_pipeline:
			frames = self.depth_pipeline.wait_for_frames()
			color = frames.get_color_frame()
			depth = frames.get_depth_frame()
		return color, depth
		
	def check_ground_obstacle(self, depth_array):
		bottom = depth_array[240:,200:]
		return numpy.any((bottom > 0) & (bottom < 180))


if __name__ == "__main__":
	try:
		node = RealsenseNode()
		node.execute()
	except rospy.ROSInterruptException:
		pass
