#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#  custom_action.py
#
#  author : Sreejith S
#  email  : echo $(base64 -d <<< NDQ0bGhjCg==)@gmail.com
#  date   : Thu 10 Oct 2019 04:42:10 UTC
#  ver    : 

import rospy
import actionlib
from u4_actions.msg import CustomActionMsgAction, CustomActionMsgFeedback
from std_msgs.msg import Empty

class Drone():


    """Move drone in square"""

    _feedback = TestFeedback()
    _result = TestResult()
    _cmd = Twist()
    _result.result = 0


    def __init__(self):
        """ """
        self.as_name = "drone_sq_as"
        self.sq_size = 1
        self._as = actionlib.SimpleActionServer(self.as_name, TestAction,
                                                self.goal_cb, auto_start=False)
        self._as.start()
        self.vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.takeoff_pub = rospy.Publisher("/drone/takeoff", Empty, queue_size=1)
        rospy.loginfo("/cmd_vel : Waiting for connection")
        while not self.vel_pub.get_num_connections():
            rospy.sleep(0.5)
        rospy.loginfo("/cmd_vel : Subscribers connected.")
        self.takeoff_pub.publish()
        rospy.loginfo("Drone takeoff")
        self.take_off()


    def timer_cb(self, event):
        """Calback for rospy.Timer
            Increments result(duration of action) by 1 """
        self._result.result += 1


    def goal_cb(self, goal):
        """

        """
        self._result.result = 0             #reset time taken to complete
        self.sq_size = goal.goal
        timer = rospy.Timer(rospy.Duration(1), self.timer_cb)

        success = True                      #flag
        for i in range(4):
            self._feedback.feedback = i + 1

            if self._as.is_preempt_requested():
                success = False
                rospy.logwarn("Action cancelled.")
                self._as.set_preempted()
                break

            self.move_forward()
            self.turn_ccw()
            self._as.publish_feedback(self._feedback)

        timer.shutdown()
        if success:
            rospy.loginfo("Successfully completed action")
            self._as.set_succeeded(self._result)


    def stop(self):
        self._cmd = Twist()
        self.vel_pub.publish(self._cmd)

    def turn_ccw(self):
        rospy.loginfo("Turning CCW...")
        self._cmd.linear.x = 0.0
        self._cmd.angular.z = 1.0
        self.vel_pub.publish(self._cmd)
        rospy.sleep(1.57)
        self.stop()

    def move_forward(self):
        rospy.loginfo("Moving forward...")
        self._cmd.linear.x = 1.0
        self._cmd.angular.z = 0.0
        self.vel_pub.publish(self._cmd)
        rospy.sleep(self.sq_size)
        self.stop()

    def take_off(self):
        self._cmd.linear.z = 1.0
        self.vel_pub.publish(self._cmd)
        rospy.sleep(4)
        self.stop()




if __name__ == "__main__":
    rospy.init_node("drone_in_square")
    drone = Drone()
    rospy.spin()


