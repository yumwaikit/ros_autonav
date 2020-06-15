#!/usr/bin/env python

import rospy
from autorun_core import AutorunNode


if __name__ == "__main__":
	try:
		node = AutorunNode()
		node.execute()
	except rospy.ROSInterruptException:
		pass
