#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from itertools import cycle
from random import choice
from graph_msgs.msg import GeometryGraph
import threading
from tkthread import TkThread
import itertools
import networkx as nx
import matplotlib.pyplot as plt
import numpy as np
import json

rospy.init_node("graph_subscriber")

class Readers:
    def __init__(self, robot_list, graph_topic="/random_walkers/graph"):
        self.graph = nx.Graph()
        self.robot_pose_dict = {i:Point() for i in robot_list}
        self.robot_plot_fix_pose = {i:(0, 0) for i in robot_list}
        self.graph_edges = []
        self.graph.add_nodes_from(robot_list)
        self.sub = rospy.Subscriber(graph_topic, GeometryGraph, self.odom_callback)
        self.graph_max_wt = 5 #only edge weights lesser will be considered
        #update graph every 4 secs
        # self.fig, self.ax = plt.subplots(figsize=(6,4))
        self.calculate_dist() #init graph
    def timer_thread(self):
            # rospy.Timer(rospy.Duration(10), self.calculate_dist_timer_cb, oneshot=False)
            while True:
                rospy.sleep(2)
                self.calculate_dist()

    def eucledian_dist(self, pt1, pt2):
        """Get eucledian dist from current bot to point pt"""
        p1 = np.array((pt1.x, pt1.y))
        p2 = np.array((pt2.x, pt2.y))
        return np.linalg.norm(p1-p2)

    def odom_callback(self, msg):
        self.robot_pose_dict[msg.id] = msg.node_coord
        # rospy.loginfo("Coords: {}".format(self.robot_pose_dict))

    def calculate_dist(self):
        """Calculate the distances between  and add to graph"""
        rospy.loginfo("Updating graph...")
        self.graph.remove_edges_from(list(self.graph.edges()))
        for i, v in self.robot_pose_dict.items():
            for j, u in self.robot_pose_dict.items():
                d = self.eucledian_dist(v, u)
                if d < self.graph_max_wt and i != j:
                    self.graph.add_edge(i, j, weight=d)
            self.robot_plot_fix_pose[i] = (v.x, v.y)
        rospy.loginfo("Edge list: {}".format(self.graph.edges()))
        rospy.loginfo("Connected Components: {}".format(list(nx.connected_components(self.graph))))
        # nx.write_weighted_edgelist(self.graph, "graph.txt")
        # with open("layout.txt", "w") as fp:
            # fp.write(json.dumps(self.robot_plot_fix_pose))

    def calculate_dist_timer_cb(self, event):
        self.calculate_dist()

    def plot_update(self):
        self.ax.clear()
        nx.draw_networkx(self.graph, pos=self.robot_plot_fix_pose, width=2,
                            ax=self.ax, alpha=0.7, style='solid',
                            edge_cmap=plt.cm.viridis)
        self.ax.set_title("Connected Components: {}".format(
            list(nx.connected_components(self.graph))
            ))
        # plt.show(block=True)


if __name__ == "__main__":
    robot_list = list(range(10))
    r = Readers(robot_list, graph_topic="/random_walkers/graph")
    thread = threading.Thread(target=r.timer_thread, daemon=True)
    thread.start()
    fig, ax = plt.subplots(figsize=(6,4))
    plt.ion()
    while True:
        plt.pause(3)
        ax.clear()
        ax.set_xlim([-9,9])
        ax.set_ylim([-9,9])
        nx.draw_networkx(r.graph, pos=r.robot_plot_fix_pose, width=2,
                            ax=ax, alpha=0.7, style='solid',
                            edge_cmap=plt.cm.viridis)
        ax.set_title("Connected Components: {}".format(
            list(nx.connected_components(r.graph))
            ))
        fig.canvas.draw()





