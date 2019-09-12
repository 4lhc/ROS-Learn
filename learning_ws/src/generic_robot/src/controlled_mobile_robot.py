#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#  controlled_mobile_robot.py
#
#  author : Sreejith S
#  email  : echo $(base64 -d <<< NDQ0bGhjCg==)@gmail.com
#  date   : Thu 12 Sep 2019 18:13:20 IST
#  ver    :
# A derived class for baic controls


import rospy
from math import radians
# from geometry_msgs.msg import Twist, Pose, Point, Quaternion
# from sensor_msgs.msg import LaserScan
# from nav_msgs.msg import Odometry
# from tf.transformations import euler_from_quaternion, quaternion_from_euler
# import math
from mobile_robot import MobileRobot

class ControlledMobileRobot(MobileRobot):
    def __init__(self, name="robot1",
                 cmd_vel_topic="/cmd_vel",
                 odom_topic="/odom",
                 laser_topic=None,
                 pos_tolerance=0.1,
                 ang_tolerance=0.005,
                 lin_vel=0.3, ang_vel=0.5):
        """
        ControlledMobileRobot Object - A base for building mobile robots in ROS

        """
        MobileRobot.__init__(self, name=name,
                 cmd_vel_topic=cmd_vel_topic,
                 odom_topic=odom_topic,
                 laser_topic=laser_topic,
                 pos_tolerance=pos_tolerance,
                 ang_tolerance=ang_tolerance,
                 lin_vel=lin_vel, ang_vel=ang_vel)

    #ControlledMotion----------------------------------------------------------
    def move_forward_dist(self, target_dist=1.0, use_accel=True):
        """
        Moves the robot for the specified distance in the +x direction

        Args:
            target_dist (float): The distance to move[m], (default 1.0)
            use_accel (bool): Set True if acceleration to be used (default True)

        Returns:
            True:  if moved within positional tolerance
        """

        if not use_accel:
            lx = self.get_dist_travelled()
            while (lx < target_dist):
                lx = self.get_dist_travelled()
                self.cmd.linear.x = self.max_lin_vel
                self.move()
                rospy.sleep(self.sleep_time)
            self.stop()
            return True

        l = target_dist
        #acceleration stop pos
        l1 = ((pow(self.max_lin_vel, 2) - pow(self.cmd.linear.x, 2))
              /(2*self.max_lin_accel))
        #decceleration start pos
        l2 = l - (pow(self.max_lin_vel, 2))/(2*self.max_lin_deccel)
        #expect negative value - what if distance is too short for accel to max_vel?
        lx = self.get_dist_travelled() #dist travelled so far
        while (lx < target_dist):
            lx = self.get_dist_travelled()
            if (lx > l2):
                self.curr_lin_accel = -pow(self.odom.twist.twist.linear.x, 2)/(2*(l-lx)) #BB8 is weirdly oriented
                if self.cmd.linear.x <= 0:
                    break
            elif (lx < l1):
                self.curr_lin_accel = self.max_lin_accel
            else:
                self.curr_lin_accel = 0

            self._set_vel_from_accel()
            self.cmd.angular.z = 0
            self.move()
            rospy.sleep(self.sleep_time)
            p = "{:.4f} | {:.4f} | {:.4f} | {:.4f}".format(
                    self.get_dist_travelled(),
                    self.cmd.linear.x,
                    self.odom.twist.twist.linear.x,
                    self.curr_lin_accel)
            print(p)
        return True


    def turn_angle(self, angle=radians(90), relative=True):
        """
        Turn the robot a specified angle

        Args:
            angle (float): Angle[rad] (default radians(90))
            relative (bool): If True, use relative positioning (default True)

        Returns:
            True if success
        """
        start_angle = self.get_orientation_euler()[2]
        if relative:
            target_angle = start_angle + angle
        else:
            target_angle = angle

        if target_angle > radians(180):
            target_angle -= radians(360)
        print("start: {:.4f} target: {:.4f}".format(start_angle, target_angle))


        curr_angle = self.get_orientation_euler()[2]
        while abs(target_angle - curr_angle) > self.ang_tol:
            curr_angle = self.get_orientation_euler()[2]
            if target_angle < 0:
                self.cmd.angular.z = (-1)*self.max_ang_vel
            else:
                self.cmd.angular.z = (1)*self.max_ang_vel
            self.move()
            rospy.sleep(self.sleep_time)
            print("curr: {:.4f} target: {:.4f} ang.z: {:.4f}".format(curr_angle, target_angle, self.cmd.angular.z))
        self.stop()



if __name__ == "__main__":
    rospy.init_node('test_robot')
    robot = ControlledMobileRobot(name="robot1", lin_vel=0.3)
    # robot.move_forward_dist(target_dist=5.0, use_accel=True)

    # robot.stop()

    # rospy.loginfo("curr_pose : :")
    # rospy.loginfo(robot.get_curr_pose())
    # rospy.loginfo("dist_travelled : : ")
    # rospy.loginfo(robot.get_dist_travelled())
    # rospy.loginfo("curr_position : : ")
    # rospy.loginfo(robot.get_curr_position())
    # robot.set_start_pose()
    robot.turn_angle(angle=radians(45), relative=True)




