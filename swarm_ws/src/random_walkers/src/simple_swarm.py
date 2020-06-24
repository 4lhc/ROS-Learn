#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from graph_msgs.msg import GeometryGraph
from itertools import cycle
from random import choice
import threading
import itertools

rospy.init_node("simple_swarm_node")

class Robot:
    newid = itertools.count().next
    def __init__(self, name="robot1", lin_vel=0.3, ang_vel=0.5):
        self.name = name
        self.id = Robot.newid()
        self.graph_publishing = False
        self.graph_publisher = None
        rospy.loginfo("Robot {} created with id: {}".format(self.name, self.id))
        self.cmd_vel = Twist()
        self.pose = Point()
        self.lin_vel = lin_vel
        self.ang_vel = ang_vel
        self.pub = rospy.Publisher("/{}/cmd_vel".format(name), Twist, queue_size=1)
        self.sub = rospy.Subscriber("/{}/scan".format(name), LaserScan, self.laser_callback)
        self.sub = rospy.Subscriber("/{}/odom".format(name), Odometry, self.odom_callback)
        self.scenario_lookup_table = {  0: self.move_forward,
                                        1: self.turn_left,
                                        2: self.turn_right,
                                        3: self.turn_left,
                                        4: self.turn_right,
                                        5: self.move_forward,
                                        6: self.turn_right,
                                        7: self.stop}



        while self.pub.get_num_connections() < 1:
            rospy.loginfo("Waiting for connections")
            rospy.sleep(0.2)

    def laser_callback(self, msg):
        self.dist_to_obstacle_in_front = min(min(msg.ranges[0:20]), min(msg.ranges[340:359]), 10)
        self.dist_to_obstacle_in_right = min(min(msg.ranges[315:340]), 10)
        self.dist_to_obstacle_in_left = min(min(msg.ranges[20:45]), 10)

    def odom_callback(self, msg):
        if self.graph_publishing:
            self.pose = msg.pose.pose.position
            pub_msg = GeometryGraph()
            pub_msg.id = self.id
            pub_msg.node_coord = self.pose
            self.graph_publisher.publish(pub_msg)


    def set_graph_publishing(self, pub):
        self.graph_publishing = True
        self.graph_publisher = pub

    def get_scenario(self):
        ob_lim = 1.5 #obstacle distance
        ob_f = self.dist_to_obstacle_in_front
        ob_l = self.dist_to_obstacle_in_left
        ob_r = self.dist_to_obstacle_in_right
        scenario = [4*(ob_l<ob_lim), 2*(ob_f<ob_lim), 1*(ob_r<ob_lim)]
        case = sum(scenario)
        rospy.loginfo("{} : Case = {} , Scenario = {}".format(self.name,
                                                              case,
                                                              scenario))
        return case

    def obstacle_avoidance(self):
        case = self.get_scenario()
        cmd = self.scenario_lookup_table[case]
        cmd()


    def move(self):
        self.pub.publish(self.cmd_vel)

    def turn_right(self):
        '''cw'''
        rospy.loginfo("Turning right")
        self.cmd_vel.angular.z = -abs(self.ang_vel)
        self.move()

    def turn_left(self):
        '''ccw'''
        rospy.loginfo("Turn Left")
        self.cmd_vel.angular.z = abs(self.ang_vel)
        self.move()

    def move_forward(self):
        rospy.loginfo("Moving forward")
        self.cmd_vel.angular.z = 0
        self.cmd_vel.linear.x = self.lin_vel
        self.move()

    def stop(self):
        rospy.loginfo("Stoping...")
        self.cmd_vel.angular.z = 0
        self.cmd_vel.linear.x = 0
        self.move()



class StoppableThread(threading.Thread):
    def __init__(self , target):
        super(StoppableThread, self).__init__(target=target)
        self._stop_event = threading.Event()

    def stop(self):
        rospy.loginfo("Thread stop received")
        self._stop_event.set()

    def is_stopped(self):
        return self._stop_event.is_set()




class RandomWalker(Robot):
    def __init__(self, name="robot1", lin_vel=0.75, ang_vel=1):
        #super().__init__(name, lin_vel, ang_vel)
        Robot.__init__(self, name, lin_vel, ang_vel)
        self.random_cmd_list = [self.move_forward,
                                self.turn_left,
                                self.turn_right,
                                self.stop]
        self.random_cmd_list.extend([self.move]*6)
        self.stop_counter = cycle(range(3)) #auto resetting counter
        self.thread = StoppableThread(target=self.obstacle_avoidance)
        self.sleep_duration = 0.3

    def __del__(self):
        self.thread.stop()
        rospy.loginfo("Robot deleted")
        self.thread.join()
        self.stop()

    def reverse(self):
        '''override of stop - reverse in RandomWalker'''
        rospy.loginfo("Reversing...")
        self.cmd_vel.angular.z = 0
        self.cmd_vel.linear.x = -self.lin_vel
        self.move()

    def obstacle_avoidance(self):
        while True:
            case = self.get_scenario()
            if case == 0:
                cmd = choice(self.random_cmd_list) #no obstacle - do random walk
            else:
                cmd = self.scenario_lookup_table[case]
            cmd()
            #to detect if the last 3 cmds were stop - if so take a reverse
            if case != 7:
                self.stop_cntr = cycle(range(3)) #reset counter to 0
            elif case == 7 and next(self.stop_counter) == 2: #counter will autoreset
                self.reverse()
            rospy.sleep(self.sleep_duration)

    def start_random_walk(self):
        self.thread.start()



if __name__ == "__main__":
    graph_pub = rospy.Publisher("/random_walkers/graph", GeometryGraph, queue_size=1)
    robots = []
    for i in range(10):
        robo_name = "robot{}".format(i)
        robot = RandomWalker(name=robo_name, lin_vel=0.9, ang_vel=1)
        robot.start_random_walk()
        robots.append(robot)
        robot.set_graph_publishing(graph_pub)
    try:
        i = 10
        while i > 0:
            rospy.sleep(1)
            i -= 1

    except Exception as err:
        [robot.__del__() for robot in robots]
        rospy.info(err)
    finally:
        [robot.__del__() for robot in robots]




