#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#  test.py
#
#  author : Sreejith S
#  email  : echo $(base64 -d <<< NDQ0bGhjCg==)@gmail.com
#  date   : Fri 23 Aug 2019 01:23:38 UTC
#  ver    :


import rospy
from U3_services.srv import CustomServiceMessage, CustomServiceMessageResponse


rospy.init_node('simple_service_server')


def service_callback(req):
    callerid = req._connection_header['callerid']
    print("Request from {} Received\nName: {}".format(callerid, req.name))
    resp = CustomServiceMessageResponse()
    resp.name_len = len(req.name)
    print("Response \nName Length: {}".format(resp.name_len))
    return resp


custom_service = rospy.Service('/get_name_len', CustomServiceMessage, service_callback)
rospy.spin()

