#!/usr/bin/env python

import numpy as np
import rospy
import pyrealsense2 as rs
from std_msgs.msg import Bool
from rospy.numpy_msg import numpy_msg

def check_ground_obstacle(depth_array):
	bottom = depth_array[240:,200:]
	return np.any((bottom > 0) & (bottom < 180))

def grab_frame(pipeline):
	frames = pipeline.wait_for_frames()
	color = frames.get_color_frame().as_frame().get_data()
	color_array = np.asanyarray(color)
	depth = frames.get_depth_frame().as_frame().get_data()
	depth_array = np.asanyarray(depth)
	return color_array, depth_array

def realsense_node():
	depth_publisher = rospy.Publisher('rs_depth', Bool, queue_size = 1)
	color_publisher = rospy.Publisher('rs_color', numpy_msg(Int32), queue_size = 1)
	rospy.init_node('realsense_node')
	r = rospy.Rate(10)
	
	pipeline = rs.pipeline()
	pipeline.start()
	
	while not rospy.is_shutdown():
		color_array, depth_array = grab_frame(pipeline)
		ok = check_ground_obstacle(depth_array)
		if ok == True:
			print('Low obstacle')
		else:
			print('Clear')
		depth_publisher.publish(ok)
		color_publisher.publish(color_array)
		r.sleep()

if __name__=="__main__":
	try:
		realsense_node()
	except rospy.ROSInterruptException:
		pass
