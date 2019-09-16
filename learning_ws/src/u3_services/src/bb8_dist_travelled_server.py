#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#  bb8_dist_travelled_server.py
#
#  author : Sreejith S
#  email  : echo $(base64 -d <<< NDQ0bGhjCg==)@gmail.com
#  date   : Sun 15 Sep 2019 15:24:26 UTC
#  ver    : 

import rospy
from u3_services.srv import MyDist, MyDistResponse
from nav_msgs.msg import Odometry
from controlled_mobile_robot import ControlledMobileRobot


class Robot(ControlledMobileRobot):
    def __init__(self):
        ControlledMobileRobot.__init__(self, name="bb8", odom_topic="/odom")
        self.dist_service = rospy.Service('/get_dist_travelled', MyDist, self.service_callback)

    def service_callback(self, request):
        if request.units.lower() in ["miles", "mi", "mile"]:
            conversion_factor = 1/1609.344
        elif request.units.lower() in ["meters", "m", "mts", "meter"]:
            conversion_factor = 1
        dist = conversion_factor*self.get_dist_travelled()
        return MyDistResponse(dist=dist)


if __name__ == "__main__":
    rospy.init_node('bb8_dist_travelled')
    robot = Robot()
    while True:
        robot.move_forward_dist(target_dist=5.0, use_accel=True)

