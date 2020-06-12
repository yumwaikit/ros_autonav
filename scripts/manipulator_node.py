#!/usr/bin/env python

import math
import rospy
from std_msgs.msg import Float64MultiArray


def manipulator_node():
	joint_publisher = rospy.Publisher('joint_trajectory_point', Float64MultiArray, queue_size=10)
	gripper_publisher = rospy.Publisher('gripper_position', Float64MultiArray, queue_size=10)
	rospy.init_node('manipulator_node')
	r = rospy.Rate(1)

	for i in range(3):
		joints = Float64MultiArray()
		joints.data = [1.0, 0.0, -math.pi / 2, math.pi * 3 / 8, math.pi / 4]
		gripper = Float64MultiArray()
		gripper.data = [0.0]
		joint_publisher.publish(joints)
		gripper_publisher.publish(gripper)
		r.sleep()


if __name__ == "__main__":
	try:
		manipulator_node()
	except rospy.ROSInterruptException:
		pass
