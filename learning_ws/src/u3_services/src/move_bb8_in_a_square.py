#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#  move_bb8_in_a_square.py
#
#  author : Sreejith S
#  email  : echo $(base64 -d <<< NDQ0bGhjCg==)@gmail.com
#  date   : Thu 16 Sep 2019 09:40:52 UTC
#  ver    : 
# Service Server node to run bb8 in a square

import rospy
from controlled_mobile_robot import ControlledMobileRobot
from u3_services.srv import BB8CustomServiceMessage, BB8CustomServiceMessageResponse
from math import radians


rospy.init_node("move_bb8_in_a_square")
robot = ControlledMobileRobot(name="bb8", odom_topic="/odom")


def move_square(side, reps):
    try:
        for i in range(4*reps):
            robot.move_forward_dist(target_dist=side, use_accel=True)
            robot.stop()
            rospy.sleep(1)
            robot.set_start_pose()
            print(robot.max_ang_vel)
            robot.turn_cw(ang_vel=0.5)
            print(radians(45)/robot.max_ang_vel)
            rospy.sleep(radians(45)/robot.max_ang_vel) #sleep to turn 90 (45??!?) deg
            robot.stop()
            rospy.sleep(1)
            robot.set_start_pose()
            rospy.sleep(1)
        return True
    except Exception as err:
        rospy.log(err)
        return False

def service_callback(request):
    status = False
    if move_square(request.side, request.repetitions):
        status = True
    return BB8CustomServiceMessageResponse(success=status)

sq_service = rospy.Service('/move_bb8_in_square_custom', BB8CustomServiceMessage, service_callback)



#reset Gazebo
from std_srvs.srv import Empty
gz_reset_world = rospy.ServiceProxy('/gazebo/reset_world', Empty)
gz_reset_sim = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
gz_reset_sim()
gz_reset_world()
rospy.spin()
