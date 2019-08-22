#!/usr/bin/env python
# author: Sreejith S
# 20-Aug-2019
# Curses based python dashboard to display rostopics
# TODO: make general purpose to accept any topic

import curses
import rospy
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from std_srvs.srv import Empty

rospy.init_node('curses_dashboard')
reset_simulation = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)

screen = curses.initscr()
# screen.keypad(1)
# curses.noecho()
# screen.nodelay(1)
# curses.start_color()
num_rows, num_cols = screen.getmaxyx()

screen.border(0)
cmd_vel_win = curses.newwin(num_rows, num_cols/3, 0, 0)
cmd_vel_win.addstr(1, 1, "/cmd_vel")
cmd_vel_win.border()
cmd_vel_win.refresh()


odom_pose_win = curses.newwin(num_rows, num_cols/3, 0, num_cols/3)
odom_pose_win.addstr(1, 0, "/odom/pose/pose")
odom_pose_win.border()
odom_pose_win.refresh()

odom_twist_win = curses.newwin(num_rows, num_cols/3, 0, 2*num_cols/3)
odom_twist_win.addstr(1, 0, "/odom/twist/twist")
odom_twist_win.border()
odom_twist_win.refresh()

def twist_print(win, msg, title):
    win.erase()
    win.addstr(1, 1, title)
    win.addstr(2, 1, "linear:")
    win.addstr(3, 2, "x: {}".format(msg.linear.x))
    win.addstr(4, 2, "y: {}".format(msg.linear.y))
    win.addstr(5, 2, "z: {}".format(msg.linear.z))
    win.addstr(6, 1, "angular:")
    win.addstr(7, 2, "x: {}".format(msg.angular.x))
    win.addstr(8, 2, "y: {}".format(msg.angular.y))
    win.addstr(9, 2, "z: {}".format(msg.angular.z))
    win.border()
    win.refresh()


def cmd_vel_callback(msg):
    twist_print(cmd_vel_win, msg, "/cmd_vel")


def odom_callback(msg):
    oq = msg.pose.pose.orientation
    pos = msg.pose.pose.position
    or_q = [oq.x, oq.y, oq.z, oq.w]
    (r, p, y) = euler_from_quaternion(or_q)
    odom_pose_win.erase()
    odom_pose_win.addstr(1, 1, "/odom/pose/pose")
    odom_pose_win.border()
    odom_pose_win.addstr(2,1,"position:")
    odom_pose_win.addstr(3,2,"x: {}".format(pos.x))
    odom_pose_win.addstr(4,2,"y: {}".format(pos.y))
    odom_pose_win.addstr(5,2,"z: {}".format(pos.z))
    odom_pose_win.addstr(6,1,"orientation:")
    odom_pose_win.addstr(7,2,"roll:  {}".format(r))
    odom_pose_win.addstr(8,2,"pitch: {}".format(p))
    odom_pose_win.addstr(9,2,"yaw:   {}".format(y))
    odom_pose_win.refresh()

    twist_print(odom_twist_win, msg.twist.twist, "/odom/twist/twist")

def shutdown_hook():
    curses.endwin()

sub_cmd_vel = rospy.Subscriber('/cmd_vel', Twist, cmd_vel_callback)
sub_odom = rospy.Subscriber('/odom', Odometry, odom_callback)
# sub_odom_pose = rospy.Subscriber('/odom', Pose, odom_pose_callback)
# sub_odom_twist = rospy.Subscriber('/odom', Odometry, odom_twist_callback)
rospy.on_shutdown(shutdown_hook)


rospy.spin()
# while True:
    # if screen.getch() == ord('r'):
        # print(key)
        # reset_simulation() #resets simulation if 'r' is pressed




