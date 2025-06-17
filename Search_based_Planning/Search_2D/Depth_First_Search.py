import os
import sys
import math
from collections import deque

sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
                "/../../Search_based_Planning/")

from Search_2D import plotting, env

class DFS():
    def __init__(self, start_node, end_node, cost_mode):
        self.start_node = start_node
        self.end_node = end_node
        self.cost_mode = cost_mode
        self.env = env.Env()

        self.OpenList = deque()
        self.CloseList = deque()

        self.ParentDict = dict()
        self.CostDict = dict()

        self.PathList = []
        self.VistList = []

    def search_path(self):
        self.OpenList.append(self.start_node)
        self.ParentDict[self.start_node] = None
        self.CostDict[self.start_node] = 0

        while self.OpenList:
            node_cur = self.OpenList.pop()
            if node_cur in self.CloseList:
                continue
            else:
                self.CloseList.append(node_cur)
                if node_cur == self.end_node:
                    break
                else:
                    for node_nb in self.get_neighbor(node_cur):
                        if node_nb in self.CloseList:
                            continue
                        else:
                            nb_cost = self.CostDict[node_cur] + self.get_cost(node_cur, node_nb)
                            if node_nb in self.OpenList:
                                if nb_cost < self.CostDict[node_nb]:
                                    self.CostDict[node_nb] = nb_cost
                                    self.ParentDict[node_nb] = node_cur
                            else:
                                self.OpenList.append(node_nb)
                                self.ParentDict[node_nb] = node_cur
                                self.CostDict[node_nb] = nb_cost


    def get_neighbor(self, node_cur):
        neighbor = []
        for motion in self.env.motions:
            node_nb = (node_cur[0] + motion[0], node_cur[1] + motion[1])
            if self.judge_obstacle(node_cur, node_nb):
                continue
            else:
                neighbor.append(node_nb)
        return neighbor
    
    def judge_obstacle(self, node_cur, node_nb):
        node_xmin_ymin = (min(node_cur[0], node_nb[0]), min(node_cur[1], node_nb[1]))
        node_xmin_ymax = (min(node_cur[0], node_nb[0]), max(node_cur[1], node_nb[1]))
        node_xmax_ymin = (max(node_cur[0], node_nb[0]), min(node_cur[1], node_nb[1]))
        node_xmax_ymax = (max(node_cur[0], node_nb[0]), max(node_cur[1], node_nb[1]))

        if any(node in self.env.obs for node in [node_xmin_ymin, node_xmin_ymax, node_xmax_ymin, node_xmax_ymax]):
            return True
        else:
            return False
        
    def get_cost(self, node_cur, node_nb):
        if self.cost_mode == 'Const':
            return 1
        elif self.cost_mode == 'Euclidean':
            return math.sqrt((node_nb[0] - node_cur[0])**2 + (node_nb[1] - node_cur[1])**2)
        elif self.cost_mode == 'Manhattan':
            return abs(node_nb[0] - node_cur[0]) + abs(node_nb[1] - node_cur[1])
        
    def get_path(self):
        node = self.CloseList[-1]
        while node:
            self.PathList.append(node)
            node = self.ParentDict[node]

    def get_visit(self):
        self.VistList = list(self.CloseList)


def main():
    s_start = (5, 5)
    s_goal = (45, 25)

    dfs = DFS(s_start, s_goal, 'Const')
    dfs.search_path()
    dfs.get_path()
    dfs.get_visit()

    if dfs.PathList[0] == dfs.end_node:
        print("Path found!")
        print(f"Path length: ", len(dfs.PathList))
        print(f"Visited nodes: ", len(dfs.VistList))
        print(f"Cost:", dfs.CostDict[dfs.PathList[0]])
    else:
        print("Path not found!")
    
    plot = plotting.Plotting(s_start, s_goal)
    plot.animation(dfs.PathList, dfs.VistList, "Depth-first Searching (DFS)")  # animation


if __name__ == '__main__':
    main()