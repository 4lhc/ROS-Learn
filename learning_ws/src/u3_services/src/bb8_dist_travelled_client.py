#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#  bb8_dist_travelled_client.py
#
#  author : Sreejith S
#  email  : echo $(base64 -d <<< NDQ0bGhjCg==)@gmail.com
#  date   : Sun 15 Sep 2019 16:17:09 UTC
#  ver    : 

import rospy
from u3_services.srv import MyDist, MyDistRequest

srv_name = '/get_dist_travelled'
dist_service_client = rospy.ServiceProxy(srv_name, MyDist)
rospy.wait_for_service(srv_name)
dist_resp = dist_service_client(units="m")
print(dist_resp)
