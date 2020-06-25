import numpy as np
import networkx as nx
import matplotlib.pyplot as plt
from matplotlib import animation
from random import choice

def rand_edge(n=5):
    """Test anim"""
    # robot_list = [f"R_{i}" for i in range(1, 11)]
    robot_list = list(range(10))
    e_parent = np.random.choice(robot_list, n)
    e_child = np.random.choice(robot_list, n)
    return [e for e in zip(e_parent, e_child)]

def simple_update(num, n, layout, G, ax):
    ax.clear()
    # Draw the graph with random node colors
    random_colors = np.random.randint(2, size=n)
    edges = rand_edge()
    G.remove_edges_from(list(G.edges()))
    G.add_edges_from(edges)
    fixed_positions = {0:(-1,1),1:(0,1),2:(1,1),
                       3:(-1,0),4:(0,0),5:(1,0),
                       6:(-1,-1),7:(0,-1),8:(1,-1),9:(1,-2)}
    nx.draw_networkx(G, pos=layout, width=2,
                        ax=ax, alpha=0.7, style='solid',
                        edge_cmap=plt.cm.viridis)
    # nx.draw_networkx_nodes(G, pos=layout, node_color=random_colors, ax=ax)
    # nx.draw_networkx_edges(G, pos=layout, edge_color=random_colors, ax=ax)
    # Set the title
    ax.set_title("Frame {}".format(num))


def simple_animation():
    # robot_list = [f"R_{i}" for i in range(1, 11)]
    robot_list = list(range(10))
    # Build plot
    fig, ax = plt.subplots(figsize=(6,4))
    # Create a graph and layout
    n = len(robot_list) # Number of nodes
    m = 9 # Number of edges
    G = nx.Graph()
    G.add_nodes_from(robot_list)
    # layout = nx.spring_layout(G)
    layout = {0:(-1,1),1:(0,1),2:(1,1),
                       3:(-1,0),4:(0,0),5:(1,0),
                       6:(-1,-1),7:(0,-1),8:(1,-1),9:(1,-2)}
    ani = animation.FuncAnimation(fig, simple_update, frames=10, fargs=(n, layout, G, ax))
    # ani.save('animation_1.gif', writer='imagemagick')

    plt.show()

simple_animation()
