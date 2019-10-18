#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#  fibonacci.py
#
#  author :
#  email  :
#  date   : Tue 08 Oct 2019 10:21:54 IST
#  ver    : 
import rospy
import actionlib
from actionlib_tutorials.msg import FibonacciFeedback, FibonacciResult, FibonacciAction


class FibonacciClass(object):

    """Class def for Fibonacci action server"""
    _feedback = FibonacciFeedback()
    _result = FibonacciResult()


    def __init__(self):
        """creates the action server """
        self.as_name = "fibonacci_as"
        self._as = actionlib.SimpleActionServer(self.as_name, FibonacciAction,
                self.goal_callback, auto_start=False)
        # @param  auto_start A boolean value that tells the ActionServer wheteher
        # or not to start publishing as soon as it comes up.
        # THIS SHOULD ALWAYS BE SET TO FALSE TO AVOID RACE CONDITIONS and start()
        # should be called after construction of the server.
        self._as.start()

    def goal_callback(self, goal):
        """Computes the fibonacci sequence and returns it to the node that called
            the action server
        """
        r = rospy.Rate(1)
        success = True


        self._feedback.sequence = [0, 1]

        rospy.loginfo("{} : Create fibonacci of order {}".format(self.as_name,
                                                                 goal.order))
        for i in range(1, goal.order):
            if self._as.is_preempt_requested():
                rospy.loginfo("The goal has been cancelled")
                self._as.set_preempted()
                success = False
                break

            self._feedback.sequence.append( self._feedback.sequence[i] +
                                            self._feedback.sequence[i-1])

            self._as.publish_feedback(self._feedback)

            r.sleep()


        if success:
            self._result.sequence = self._feedback.sequence
            rospy.loginfo("Fibonacci order successfull calculated")
            self._as.set_succeeded(self._result)

if __name__ == "__main__":
    rospy.init_node('fibonacci')
    FibonacciClass()
    rospy.spin()






