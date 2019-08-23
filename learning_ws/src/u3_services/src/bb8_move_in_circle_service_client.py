#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#  bb8_circle_client.py
#
#  author : Sreejith S
#  email  : echo $(base64 -d <<< NDQ0bGhjCg==)@gmail.com
#  date   : Fri 23 Aug 2019 02:07:58 UTC
#  ver    : 
#  Client to call /move_bb8_in_circle service

import rospy
from std_srvs.srv import Empty, EmptyRequest

rospy.init_node('move_bb8_service_client')

serv_name = '/move_bb8_in_circle'
rospy.loginfo("Waiting for service %s", serv_name)
rospy.wait_for_service('/move_bb8_in_circle')
rospy.loginfo("%s service available", serv_name)
move_srv = rospy.ServiceProxy(serv_name, Empty)

move_srv()

