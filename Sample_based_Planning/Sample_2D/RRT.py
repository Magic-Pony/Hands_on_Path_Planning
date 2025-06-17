import os
import sys
import math
import numpy as np

sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
                "/../")

from Sample_2D import env, plotting, utils

class Node():
    def __init__(self, node_cord):
        self.x = node_cord[0]
        self.y = node_cord[1]
        self.parent = None

class RRT():
    def __init__(self, start_node, end_node, iter_max, sample_rate, step_len):
        self.env = env.Env()
        self.utils = utils.Utils()
        self.plotting = plotting.Plotting(start_node, end_node)

        self.start_node = Node(start_node)
        self.end_node = Node(end_node)
        self.iter_max = iter_max
        self.sample_rate = sample_rate
        self.step_len = step_len

        self.CostDict = dict()
        self.CostDict[self.start_node] = 0
        self.NodeTree = [self.start_node]
        self.PathList = []

    def expand_path(self):
        for i in range(self.iter_max):
            node_rd = self.sample_node()
            node_nb = self.get_neighbor(node_rd)
            node_new = self.get_new_node(node_nb, node_rd)

            if not self.utils.is_collision(node_nb, node_new):
                self.NodeTree.append(node_new)
                dist, _ = self.get_dist_theta(node_new, self.end_node)

                if dist <= self.step_len:
                    self.end_node.parent = node_new
                    self.CostDict[self.end_node] = self.CostDict[node_new] + dist
                    return self.get_path()
        return None
    
    def sample_node(self):
        if np.random.random() > self.sample_rate:
            node_rd = self.end_node
        else:
            node_rd = Node(
                (
                    np.random.uniform(self.env.x_range[0]+self.utils.delta, self.env.x_range[1]-self.utils.delta),
                    np.random.uniform(self.env.y_range[0]+self.utils.delta, self.env.y_range[1]-self.utils.delta),
                )
            )
        return node_rd
    
    def get_neighbor(self, node_rd):
        return self.NodeTree[np.argmin([self.get_dist_theta(node, node_rd)[0] for node in self.NodeTree])]

    def get_dist_theta(self, node_start, node_end):
        dx = node_end.x - node_start.x
        dy = node_end.y - node_start.y
        dist = math.hypot(dx, dy)
        theta = math.atan2(dy, dx)

        return dist, theta
    
    def get_new_node(self, node_start, node_end):
        dist, theta = self.get_dist_theta(node_start, node_end)

        step_len = min(self.step_len, dist)

        node_new = Node(
            (
                node_start.x + step_len * math.cos(theta),
                node_start.y + step_len * math.sin(theta)
            )
        )
        node_new.parent = node_start
        self.CostDict[node_new] = self.CostDict[node_start] + step_len

        return node_new
    
    def get_path(self):
        node = self.end_node
        while node:
            self.PathList.append((node.x, node.y))
            node = node.parent
        
        return self.PathList

    

def main():
    x_start = (2, 2)  # Starting node
    x_goal = (49, 24)  # Goal node

    rrt = RRT(x_start, x_goal, 10000, 0.95, 0.5)
    path = rrt.expand_path()

    if path:
        print(f'Cost: {rrt.CostDict[rrt.end_node]}')
        rrt.plotting.animation(rrt.NodeTree, rrt.PathList, "RRT", True)
    else:
        print("No Path Found!")


if __name__ == '__main__':
    main()
