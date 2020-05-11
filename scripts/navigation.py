#!/usr/bin/env python

import math
import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

class Navigation:
    def __init__(self, target_x, target_y):
        self.target_x = target_x
        self.target_y = target_y
        self.initialized = False

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

    def navigate(self, laser_scan, pose):
        dy = self.target_y - pose.position.y
        dx = self.target_x - pose.position.x
        print('remaining: (' + str(dy) + ', ' + str(dx) + ')')
        if dy < 0.05 and dx < 0.05:
            return 10 #halt and break
        else:
            angle = math.atan(dy / dx) - math.acos(pose.orientation.w)
            print('angle: ' + str(angle))
            if angle < -0.05 and not self.check_obstacle(laser_scan, 0.3, len(laser_scan.ranges) * 3 / 4, 45):
                return 3 #turn_right
            elif angle > 0.05 and not self.check_obstacle(laser_scan, 0.3, len(laser_scan.ranges) / 4, 45):
                return 2 #turn_left
            elif self.check_obstacle(laser_scan, 0.3, 0, 20):
                if self.check_obstacle(laser_scan, 0.3, len(laser_scan.ranges) * 3 / 4, 45):
                    return 2 #turn_left
                elif self.check_obstacle(laser_scan, 0.3, len(laser_scan.ranges) / 4, 45):
                    return 3 #turn_right
                else:
                    return 0 #halt
            else:
                return 1 #forward

    def wait_next(self):
        try:
            laser_scan = rospy.wait_for_message('scan', LaserScan, 0.5)
            odometry = rospy.wait_for_message('odom', Odometry, 0.5)
            if not self.initialized:
                self.target_x = self.target_x + odometry.pose.pose.position.x
                self.target_y = self.target_y + odometry.pose.pose.position.y
                self.initialized = True
            return self.navigate(laser_scan, odometry.pose.pose)
        except rospy.exceptions.ROSException:
            rospy.loginfo('Receive info timeout')
            return 0
