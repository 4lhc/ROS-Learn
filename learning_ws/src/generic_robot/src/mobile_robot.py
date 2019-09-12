#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#  mobile_robot.py
#
#  author : Sreejith S
#  email  : echo $(base64 -d <<< NDQ0bGhjCg==)@gmail.com
#  date   : Thu 05 Sep 2019 08:40:52 UTC
#  ver    :
# A Robot base class for mobile robots

import rospy
from geometry_msgs.msg import Twist, Pose, Point, Quaternion
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math

class MobileRobot:
    def __init__(self, name="robot1",
                 cmd_vel_topic="/cmd_vel",
                 odom_topic="/odom",
                 laser_topic=None,
                 pos_tolerance=0.1,
                 ang_tolerance=0.005,
                 lin_vel=0.3, ang_vel=0.5):
        """
        MobileRobot Object - A base for building mobile robots in ROS

        Args:
           name (str): Name of the robot, optional (default robot1)
           cmd_vel_topic (str): Publisher topic for Twist, optional (default /cmd_vel)
           odom_topic (str): Subscriber topic for Odometry, optional (default /odom)
           laser_topic (str): Subscriber topic for LaserScan, optional (default None)
           pos_tolerance (float): Desired positional tolerance[m], optional (defualt 0.1)
           ang_tolerance (float): Desired angular tolerance[rads], optional (defualt 0.005)
           lin_vel (float): Default linear velocity[m/s], optional (default 0.3)
           ang_vel (float): Default angular velocity[rad/s], optional (default 0.5)

        Attributes:
            cmd (geometry_msgs/Twist): Store the current desired velocities
            odom (nav_msgs/Odometry): Store the latest Odometry readings
            sleep_time (float): Sleep duration
            start_pos (geometry_msgs/Pose): Store the starting pose of the current leg
        """

        self.name = name
        self.cmd_vel_topic = cmd_vel_topic
        self.laser_topic = laser_topic
        self.odom_topic = odom_topic
        self.orientation_euler = (0.0, 0.0, 0.0) #rads (roll, pitch, yaw)
        self.pos_tol = pos_tolerance
        self.ang_tol = ang_tolerance
        rospy.loginfo("Robot {} created".format(self.name))

        self.cmd = Twist()
        self.odom = Odometry()
        self.lin_vel = lin_vel
        self.ang_vel = ang_vel
        self.sleep_time = 0.33
        # self.pub_rate = rospy.Rate(2) #2Hz-publish to /cmd_vel twice every sec
        self.cmd_vel_pub = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=1)

        self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback)
        if self.laser_topic:
            #use ony if enabled
            self.laser_msg = LaserScan()
            self.laser_sub = rospy.Subscriber(self.laser_topic, LaserScan, self.laser_callback)

        self.start_pose = Pose() #keep convention - pose = Pose & pos = Point


        rospy.loginfo("Waiting for connections")
        while self.cmd_vel_pub.get_num_connections() < 1:
            rospy.sleep(0.2)

    #Callbacks-----------------------------------------------------------------
    def laser_callback(self, msg):
        self.laser_msg = msg

    def odom_callback(self, msg):
        self.odom_msg = msg
        oq = msg.pose.pose.orientation #orientaiton in quaternion
        self.orientation_quaternion = [oq.x, oq.y, oq.z, oq.w]
        self.orientation_euler = euler_from_quaternion(self.orientation_quaternion)


    #Helpers-------------------------------------------------------------------
    def _eucledian_dist(self, start_pos, end_pos):
        """
        Calculate the eucledian distance between 2 points

        Args:
            start_pos (geometry_msgs/Point): The intial position
            end_pos (geometry_msgs/Point): The final position

        Returns:
            d (float): The calculated distance[m]

        """

        d = math.sqrt(math.pow((end_pos.x - start_pos.x), 2) +
                      math.pow((end_pos.y - start_pos.y), 2))
        return d

    def _measure_ang(self, start_pos, end_pos):
        """
        Calculate the angle joining the two points and the positive x axis

        Args:
            start_pos (geometry_msgs/Point): The intial position
            end_pos (geometry_msgs/Point): The final position

        Returns:
            theta ()float): The calculated angle[rad]

        """
        theta = math.atan2(end_pos.y - start_pos.y, end_pos.x - start_pos.x)
        #theta = math.atan((end_pos.y - start_pos.y)/(end_pos.x - start_pos.x))
        return theta

    def _wait_till_stop(self):
        """
        Waits till the robot has stopped moving completely

        The robot doesn't stop immediately after the stop() method has been called.
        This method is a helper to wait for a complete halt.

        Args:
            None

        Returns:
            None
        """
        prev_pos = self.get_curr_position()
        rospy.sleep(self.sleep_time)
        curr_pos = self.get_curr_position()
        while (self._eucledian_dist(prev_pos, curr_pos) > self.pos_tol):
            #wait till positional tolerance achieved
            rospy.sleep(self.sleep_time)
            prev_pos = curr_pos
            curr_pos = self.get_curr_position()


    #Getters-------------------------------------------------------------------
    def get_orientation_euler(self):
        '''Returns: tuple (roll, pitch, yaw)'''
        return self.orientation_euler

    def get_curr_pose(self):
        '''Returns: geomtery_msgs/Pose'''
        return self.odom_msg.pose.pose

    def get_curr_position(self):
        '''Returns: geometry_msgs/Point'''
        return self.odom_msg.pose.pose.position

    def get_dist_to(self, target_pos):
        '''
        Args: geometry_msgs/Point target_pos
        Returns: float d - distance [m]
        '''
        d = self._eucledian_dist(self.get_curr_position(), target_pos)
        rospy.loginfo("Dist to {} = {}".format(target_pos, d))
        return d

    def get_dist_travelled(self):
        '''
        get distance travelled in the current leg
        Args: geometry_msgs/Point target_pos
        Returns: float d - distance [m]
        '''
        sp = self.start_pose.position
        d = self._eucledian_dist(sp, self.get_curr_position())
        return d

    def get_steering_angle_to(self, target_pos):
        '''
        Args: geometry_msgs/Point target_pos
        Returns: float theta - angle [rads]

        '''
        sp = self.start_pose.position
        theta = self._measure_ang(sp, target_pos)
        rospy.loginfo("Steering Angle to {} = {}".format(sp, theta))
        return theta

    #Setters-------------------------------------------------------------------
    def set_velocities(self, lx=0.0, az=0.0):
        '''set velocities'''
        self.cmd = Twist()
        self.cmd.linear.x = lx
        self.cmd.angular.z = az

    def set_start_pose(self):
        '''Set the currrent pose as the start pose'''
        self.start_pose = self.get_curr_pose()

    #Motion--------------------------------------------------------------------
    def move(self):
        self.cmd_vel_pub.publish(self.cmd)

    def turn_cw(self, ang_vel=None):
        '''cw'''
        rospy.loginfo("Turning right")
        if not ang_vel: ang_vel = self.ang_vel
        self.cmd.angular.z = -abs(ang_vel)
        self.move()

    def turn_ccw(self, ang_vel=None):
        '''ccw'''
        rospy.loginfo("Turn Left")
        if not ang_vel: ang_vel = self.ang_vel
        self.cmd.angular.z = abs(ang_vel)
        self.move()

    def move_forward(self):
        rospy.loginfo("Moving forward")
        self.cmd.angular.z = 0
        self.cmd.linear.x = self.lin_vel
        self.move()

    def stop(self, wait=True):
        '''
        Args: wait - bool - if set True the function will wait till stop.
        Returns:
        '''
        rospy.loginfo("Stoping...")
        self.cmd.angular.z = 0
        self.cmd.linear.x = 0
        self.move()
        if wait:
            self._wait_till_stop()
            rospy.loginfo("Stopped...")

    #Actions-------------------------------------------------------------------





if __name__ == "__main__":
    rospy.init_node('test_robot')
    robot = MobileRobot(name="robot1")
    # robot.stop(wait)
    # rospy.sleep(1)
    robot.move_forward()
    rospy.sleep(2)
    robot.stop()

    rospy.loginfo("curr_pose : :")
    rospy.loginfo(robot.get_curr_pose())
    rospy.loginfo("dist_travelled : : ")
    rospy.loginfo(robot.get_dist_travelled())
    rospy.loginfo("curr_position : : ")
    rospy.loginfo(robot.get_curr_position())

    tp = Point(x=0, y=2, z=0)
    robot.get_dist_to(tp)
    robot.get_orientation_euler()
    robot.get_steering_angle_to(tp)

    robot.set_start_pose()
    robot.get_dist_travelled()



