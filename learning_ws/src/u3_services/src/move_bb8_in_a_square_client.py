#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#  move_bb8_in_a_square_client.py
#
#  author : Sreejith S
#  email  : echo $(base64 -d <<< NDQ0bGhjCg==)@gmail.com
#  date   : Fri 20 Sep 2019 16:08:58 UTC
#  ver    : 

import rospy
from u3_services.srv import BB8CustomServiceMessage, BB8CustomServiceMessageRequest

rospy.init_node('move_bb8_square_client_node')
serv_name = '/move_bb8_in_square_custom'
rospy.loginfo("Waiting for service %s", serv_name)
rospy.wait_for_service(serv_name)
rospy.loginfo("%s service available", serv_name)
move_srv = rospy.ServiceProxy(serv_name, BB8CustomServiceMessage)

two_small_sqs = BB8CustomServiceMessageRequest(side=1.0, repetitions=2)
one_larger_sq = BB8CustomServiceMessageRequest(side=2.0, repetitions=1)

rospy.loginfo("Two smaller squares")
move_srv(two_small_sqs)
rospy.loginfo("One large square")
move_srv(one_larger_sq)


