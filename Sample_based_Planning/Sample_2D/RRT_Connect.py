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

class RRT_Connect():
    def __init__(self, start_node, end_node, iter_max, sample_rate, step_len):
        self.env = env.Env()
        self.utils = utils.Utils()
        self.plotting = plotting.Plotting(start_node, end_node)

        self.start_node = Node(start_node)
        self.end_node = Node(end_node)
        self.iter_max = iter_max
        self.sample_rate = sample_rate
        self.step_len = step_len

        self.vertex1 = [self.start_node]
        self.vertex2 = [self.end_node]
        self.PathList = []

    def expand_path(self):
        for i in range(self.iter_max):
            node_sp1 = self.sample_node()
            node_near1 = self.get_nearest(self.vertex1, node_sp1)
            node_new1 = self.steer(node_near1, node_sp1)
            if not self.utils.is_collision(node_near1, node_new1):
                self.vertex1.append(node_new1)

                node_sp2 = node_new1
                node_near2 = self.get_nearest(self.vertex2, node_sp2)
                node_new2 = self.steer(node_near2, node_sp2)
                if not self.utils.is_collision(node_near2, node_new2):
                    self.vertex2.append(node_new2)

                    node_cur = node_new2
                    node_new2 = self.steer(node_cur, node_sp2)
                    while not self.utils.is_collision(node_cur, node_new2):
                        self.vertex2.append(node_new2)
                        if self.is_same(node_new2, node_sp2):
                            self.get_path()
                            return
                        else:
                            node_cur = node_new2
                            node_new2 = self.steer(node_cur, node_sp2)

            if len(self.vertex1) < len(self.vertex2):
                vertex_temp = self.vertex1
                self.vertex1 = self.vertex2
                self.vertex2 = vertex_temp

    def sample_node(self):
        if np.random.random() > self.sample_rate:
            node_sp = self.end_node
        else:
            node_sp = Node(
                (
                    np.random.uniform(self.env.x_range[0]+self.utils.delta, self.env.x_range[1]-self.utils.delta),
                    np.random.uniform(self.env.y_range[0]+self.utils.delta, self.env.y_range[1]-self.utils.delta),
                )
            )
        return node_sp
    
    def get_dist_theta(self, node_start, node_end):
        dx = node_end.x - node_start.x
        dy = node_end.y - node_start.y
        dist = math.hypot(dx, dy)
        theta = math.atan2(dy, dx)

        return dist, theta
    
    def get_nearest(self, vertex, node_sp):
        node_near = vertex[np.argmin([self.get_dist_theta(node, node_sp)[0] for node in vertex])]

        return node_near
    
    def steer(self, node_start, node_end):
        dist, theta = self.get_dist_theta(node_start, node_end)

        step_len = min(self.step_len, dist)

        node_new = Node(
            (
                node_start.x + step_len * math.cos(theta),
                node_start.y + step_len * math.sin(theta)
            )
        )
        node_new.parent = node_start

        return node_new
    
    def is_same(self, node_start, node_end):
        dist, theta = self.get_dist_theta(node_start, node_end)

        if dist <= self.step_len:
            return True
        else:
            return False
        
    def get_path(self):
        if self.vertex1[0] == self.start_node:
            start_list = self.vertex1
            end_list = self.vertex2
        else:
            start_list = self.vertex2
            end_list = self.vertex1

        start_path = []
        
        node = start_list[-1]
        while node:
            start_path.append((node.x, node.y))
            node = node.parent

        end_path = []
        node = end_list[-1]
        while node:
            end_path.append((node.x, node.y))
            node = node.parent

        for node in start_path[::-1]:
            self.PathList.append(node)
        for node in end_path:
            self.PathList.append(node)
        

def main():
    x_start = (2, 2)  # Starting node
    x_goal = (49, 24)  # Goal node

    rrt_connect = RRT_Connect(x_start, x_goal, 10000, 0.95, 0.5)
    rrt_connect.expand_path()

    rrt_connect.plotting.animation_connect(rrt_connect.vertex1, rrt_connect.vertex2, rrt_connect.PathList, "RRT_Connect")


if __name__ == '__main__':
    main()
