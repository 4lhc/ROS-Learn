#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#  bb8_move_in_circle_service_server.py
#
#  author : Sreejith S
#  email  : echo $(base64 -d <<< NDQ0bGhjCg==)@gmail.com
#  date   : Fri 23 Aug 2019 18:00:27 UTC
#  ver    : 

import rospy
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty, EmptyResponse

rospy.init_node('move_bb8_service_server')
cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
def service_callback(request):
    global cmd_vel_pub
    rospy.loginfo("Request from %s received", request._connection_header['callerid'])
    rospy.loginfo("Starting circular motion...")
    cmd_vel = Twist()
    cmd_vel.linear.x = 0.5
    cmd_vel.angular.z = 0.7
    cmd_vel_pub.publish(cmd_vel)
    return EmptyResponse()

circle_service = rospy.Service('/move_bb8_in_circle', Empty, service_callback)
rospy.spin()
