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
from U3_services.srv import DurationMessage, DurationMessageRequest

rospy.init_node('move_bb8_service_client')


rospy.wait_for_service('/move_bb8_in_circle')
duration_srv = rospy.ServiceProxy('/move_bb8_in_circle', DurationMessage)

move_req = DurationMessageRequest(duration=20)
move_resp = duration_srv(move_req)
# move_resp = duration_srv(duration=20)
print("Got Response: {}".format(move_resp))

