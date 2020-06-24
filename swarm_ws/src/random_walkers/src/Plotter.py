#!/usr/bin/env python3
import numpy as np
import networkx as nx
import matplotlib.pyplot as plt
from matplotlib import animation
from random import choice
import json
import time

def rand_edge(n=5):
    """Test anim"""
    # robot_list = [f"R_{i}" for i in range(1, 11)]
    robot_list = list(range(10))
    e_parent = np.random.choice(robot_list, n)
    e_child = np.random.choice(robot_list, n)
    return [e for e in zip(e_parent, e_child)]

class Plotter:
    """To plot animation"""
    def __init__(self):
        #match plot to sim pose
        self.graph = nx.read_weighted_edgelist("graph.txt")
        with open("layout.txt", "r") as fp:
            self.layout = json.load(fp)
        self.fig, self.ax = plt.subplots(figsize=(6,4))



    def update(self, _):
        self.graph = nx.read_weighted_edgelist("graph.txt")
        with open("layout.txt", "r") as fp:
            self.layout = json.load(fp)
        self.ax.clear()
        nx.draw_networkx(self.graph, pos=self.layout, width=2,
                            ax=self.ax, alpha=0.7, style='solid',
                            edge_cmap=plt.cm.viridis)
        self.ax.set_title("Connected Components: {}".format(
            list(nx.connected_components(self.graph))
            ))

if __name__ == "__main__":
    p = Plotter()
    anim = animation.FuncAnimation(p.fig,
                                   p.update,
                                   frames=10)
    plt.show()
