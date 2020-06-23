#!/usr/bin/env python

import math
import rospy
from std_msgs.msg import Bool
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry


class NavigationOdom:
    def __init__(self, i, offset_x, offset_y):
        while not rospy.is_shutdown():
            try:
                self.robot_id = i

                odometry = rospy.wait_for_message('/' + str(i) + '/odom', Odometry, 0.5)
                pose = odometry.pose.pose
                self.init_odo_x = pose.position.x
                self.init_odo_y = pose.position.y
                self.init_odo_w = math.acos(pose.orientation.w) * 2
                if pose.orientation.z < 0:
                    self.init_odo_w = -self.init_odo_w
                print('Robot ' + str(i) + ': Odometry initial position: (' + str(self.init_odo_x) + ', ' +
                      str(self.init_odo_y) + ')')
                print('Robot ' + str(i) + ': Odometry initial orientation: ' + str(self.init_odo_w / math.pi * 180))

                self.cur_x = 0
                self.cur_y = 0
                self.cur_w = 0
                self.offset_x = offset_x
                self.offset_y = offset_y
                self.target_x = 0
                self.target_y = 0
                self.target_w = 0
                self.turning = 0
                self.evading = 0
                print('Robot ' + str(i) + ': Auto navigation Start')
                break
            except rospy.exceptions.ROSException:
                rospy.loginfo('Robot ' + str(i) + ': Receive info timeout')

    def absolute_coordinates(self, x, y):
        init_x = self.init_odo_x
        init_y = self.init_odo_y
        w = self.init_odo_w
        new_x = y * math.sin(w) - x * math.cos(w) + init_x
        new_y = y * math.cos(w) + x * math.sin(w) + init_y
        return new_x, new_y

    def relative_coordinates(self, x, y):
        init_x = self.init_odo_x
        init_y = self.init_odo_y
        w = self.init_odo_w
        new_x = (x - init_x) * math.cos(-w) + (y - init_y) * math.sin(-w)
        new_y = (y - init_y) * math.cos(-w) - (x - init_x) * math.sin(-w)
        return new_x, new_y

    def set_target(self, target_x, target_y, target_w=0):
        self.target_x = target_x
        self.target_y = target_y
        self.target_w = target_w

    def check_obstacle(self, laser_scan, distance, index, arc_length):
        dmin = laser_scan.range_max
        for i in range(len(laser_scan.ranges)):
            rotated = (i - index + len(laser_scan.ranges)) % len(laser_scan.ranges)
            if (0 <= rotated < arc_length) or (len(laser_scan.ranges) - arc_length < rotated < len(laser_scan.ranges)):
                d = laser_scan.ranges[i]
                if laser_scan.range_min < d < laser_scan.range_max and d < dmin:
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

            if laser_scan.range_min < l < laser_scan.range_max:
                if l >= distance:
                    ok_l = ok_l + 1
                else:
                    ok_l = 0
            if laser_scan.range_min < r < laser_scan.range_max:
                if r >= distance:
                    ok_r = ok_r + 1
                else:
                    ok_r = 0

            if ok_l >= arc_length:
                self.turning = -1
                return 2  # turn_left
            if ok_r >= arc_length:
                self.turning = 1
                return 3  # turn_right

        print('No gap available')
        return 4  # turn_around

    def navigate(self, laser_scan, rs_scan, status):
        dy = self.target_y - self.cur_y - self.offset_y
        dx = self.target_x - self.cur_x - self.offset_x
        print('Robot ' + str(self.robot_id) + ': remaining: (' + str(dy) + ', ' + str(dx) + ')')

        if status == 1 and abs(dy) < 0.5 and abs(dx) < 0.5:
            return 9  # continue
        elif status == 2 and abs(dy) < 0.05 and abs(dx) < 0.05:
            angle = self.target_w - self.cur_w
            if angle < -math.pi:
                angle = angle + math.pi * 2
            elif angle > math.pi:
                angle = angle - math.pi * 2
            print('Robot ' + str(self.robot_id) + ': angle: ' + str(angle / math.pi * 180))
            if angle > 0.05:
                return 2  # turn_left
            elif angle < -0.05:
                return 3  # turn_right
            else:
                return 10  # halt and break
        else:
            angle = math.atan2(dy, dx) - self.cur_w
            if angle < -math.pi:
                angle = angle + math.pi * 2
            elif angle > math.pi:
                angle = angle - math.pi * 2
            print('Robot ' + str(self.robot_id) + ': angle: ' + str(angle / math.pi * 180))

            if self.check_obstacle(laser_scan, 0.3, 0, 30):
                if self.evading > 0 and self.turning == -1:
                    return 2  # turn_left
                elif self.evading > 0 and self.turning == 1:
                    return 3  # turn_right
                else:
                    return self.find_closest_gap(laser_scan, 0.4, 30)
            elif rs_scan:
                self.evading = 10
                if self.turning == -1:
                    return 2  # turn_left
                else:
                    return 3  # turn_right
            elif self.evading == 0 and -0.05 > angle > -math.pi / 3 \
                    and not self.check_obstacle(laser_scan, 0.3, len(laser_scan.ranges) * 3 / 4, 90):
                self.turning = 1
                return 5  # advance_right
            elif self.evading == 0 and 0.05 < angle < math.pi / 3 \
                    and not self.check_obstacle(laser_scan, 0.3, len(laser_scan.ranges) / 4, 90):
                self.turning = -1
                return 4  # advance_left
            elif self.evading == 0 and angle < -math.pi / 3 \
                    and not self.check_obstacle(laser_scan, 0.3, len(laser_scan.ranges) * 3 / 4, 90):
                self.turning = 1
                return 3  # turn_right
            elif self.evading == 0 and angle > math.pi / 3 \
                    and not self.check_obstacle(laser_scan, 0.3, len(laser_scan.ranges) / 4, 90):
                self.turning = -1
                return 2  # turn_left
            else:
                if self.evading > 0:
                    self.evading = self.evading - 1
                self.turning = 0
                return 1  # forward

    def wait_next(self, status):
        try:
            laser_scan = rospy.wait_for_message('/' + str(self.robot_id) + '/scan', LaserScan, 0.5)

            odometry = rospy.wait_for_message('/' + str(self.robot_id) + '/odom', Odometry, 0.5)
            pose = odometry.pose.pose
            self.cur_x, self.cur_y = self.relative_coordinates(pose.position.x, pose.position.y)
            self.cur_w = math.acos(pose.orientation.w) * 2
            if pose.orientation.z < 0:
                self.cur_w = -self.cur_w
            self.cur_w = self.cur_w - self.init_odo_w
            print('Odometry: (' + str(pose.position.x) + ', ' + str(pose.position.y) + ', ' +
                  str(pose.orientation.z) + ', ' + str(pose.orientation.w) + ')')

            if self.cur_w < -math.pi:
                self.cur_w = self.cur_w + math.pi * 2
            elif self.cur_w > math.pi:
                self.cur_w = self.cur_w - math.pi * 2

            print('Robot ' + str(self.robot_id) + ': Current position: (' + str(self.cur_x + self.offset_x) + ', '
                  + str(self.cur_y + self.offset_y) + ')')
            print('Robot ' + str(self.robot_id) + ': Current orientation: ' + str(self.cur_w / math.pi * 180))

            rs_scan = rospy.wait_for_message('/' + str(self.robot_id) + '/rs_depth', Bool, 0.5)
            # rs_frame = rospy.wait_for_message('rs_color', numpy_msg(Int32), 10)

            if status > 0:
                return self.navigate(laser_scan, rs_scan.data, status)
            else:
                return 0
        except rospy.exceptions.ROSException:
            rospy.loginfo('Robot ' + str(self.robot_id) + ': Receive info timeout')
            return 0
