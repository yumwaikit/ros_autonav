#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
from movements import Movement
from manipulations import Joints
from manipulations import Gripper
from navigation import Navigation

move = None
joints = None
gripper = None
nav = None
r = None

def pick_up_item():
    global joints
    global gripper
    joints.pick()
    rospy.sleep(1)
    gripper.grab()
    rospy.sleep(1)
    joints.hold()

def drop_item():
    global joints
    global gripper
    joints.pick()
    rospy.sleep(1)
    gripper.release()
    rospy.sleep(1)
    joints.neutral()

def autorun_node():
    global move
    global joints
    global gripper
    global nav
    global r
    
    move_publisher = rospy.Publisher('cmd_vel', Twist, queue_size = 10)
    joints_publisher = rospy.Publisher('joint_trajectory_point', Float64MultiArray, queue_size = 10)
    gripper_publisher = rospy.Publisher('gripper_position', Float64MultiArray, queue_size = 10)
    move = Movement(move_publisher)
    joints = Joints(joints_publisher)
    gripper = Gripper(gripper_publisher)
    nav = Navigation(1.0, 0.0)

    rospy.init_node('autorun_node')
    r = rospy.Rate(10)

    while not rospy.is_shutdown():
        output = nav.wait_next()
        if output == 0:
            print('halt')
            move.halt()
        elif output == 1:
            print('forward')
            move.forward()
        elif output == 2:
            print('turn left')
            move.turn_left()
        elif output == 3:
            print('turn right')
            move.turn_right()
        elif output == 10:
            print('exit')
            move.halt()
            break
        r.sleep()

if __name__=="__main__":
    try:
        print('start autorun')
        autorun_node()
    except rospy.ROSInterruptException:
        pass
