#!/usr/bin/env python

import rospy
from autorun_core import AutorunNode


if __name__ == "__main__":
	try:
		robot_ids = [1, 2]
		node = AutorunNode(robot_ids)
		node.execute()
	except rospy.ROSInterruptException:
		pass
