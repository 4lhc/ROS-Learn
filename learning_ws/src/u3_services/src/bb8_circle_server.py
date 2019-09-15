#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#  bb8_circle_server.py
#
#  author : Sreejith S
#  email  : echo $(base64 -d <<< NDQ0bGhjCg==)@gmail.com
#  date   : Fri 23 Aug 2019 02:07:47 UTC
#  ver    : 
#  Service which will move the bb8 in circles.
#  bb_8_gazebo and gzclient should be running.


import rospy
from geometry_msgs.msg import Twist
from u3_services.srv import DurationMessage, DurationMessageResponse

rospy.init_node('move_bb8_service_server')
cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
def service_callback(request):
    global cmd_vel_pub
    rospy.loginfo("Request from %s received", request._connection_header['callerid'])
    rospy.loginfo("Duration: %d", request.duration)
    rospy.loginfo("Starting circular motion...")
    cmd_vel = Twist()
    cmd_vel.linear.x = 0.5
    cmd_vel.angular.z = 0.7
    cmd_vel_pub.publish(cmd_vel)
    rospy.sleep(request.duration)
    cmd_vel_pub.publish(Twist())
    rospy.loginfo("Circular motion stopped.")
    return DurationMessageResponse(success=True)

circle_service = rospy.Service('/move_bb8_in_circle', DurationMessage, service_callback)
rospy.spin()
