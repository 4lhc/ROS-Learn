#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#  simple_service_client.py
#
#  author : Sreejith S
#  email  : echo $(base64 -d <<< NDQ0bGhjCg==)@gmail.com
#  date   : Fri 23 Aug 2019 01:25:37 UTC
#  ver    :

import rospy
from U3_services.srv import CustomServiceMessage, CustomServiceMessageRequest

rospy.init_node('simple_service_client')
rospy.wait_for_service('/get_name_len')

gn_service = rospy.ServiceProxy('/get_name_len', CustomServiceMessage)
robo_names = ["Rosie The Maid", "T-1000", "The Iron Gaint", "HALL 9000", "Bender", "Data"]
for name in robo_names:
    #gn_req = CustomServiceMessageRequest()
    #gn_req.name = name
    #gn_resp = gn_service(gn_req)
    gn_resp = gn_service(name=name)
    print("Got response: {}".format(gn_resp))
    rospy.sleep(2)

