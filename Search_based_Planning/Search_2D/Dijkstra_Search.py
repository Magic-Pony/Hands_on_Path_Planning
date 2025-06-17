import os
import sys
import math
from collections import deque
import heapq

sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
                "/../../Search_based_Planning/")

from Search_2D import plotting, env

class Dijkstra():
    def __init__(self, start_node, end_node, cost_mode):
        self.env = env.Env()
        self.start_node = start_node
        self.end_node = end_node
        self.cost_mode = cost_mode

        self.OpenList = []
        self.CloseList = []

        self.ParentDict = dict()
        self.CostDict = dict()

        self.PathList = []
        self.VisitList = []

    def search_path(self):
        self.ParentDict[self.start_node] = None
        self.CostDict[self.start_node] = 0
        heapq.heappush(self.OpenList, (0, self.start_node))

        while self.OpenList:
            node_cur = heapq.heappop(self.OpenList)[1]

            if node_cur not in self.CloseList:
                self.CloseList.append(node_cur)
                if node_cur == self.end_node:
                    break
                else:
                    for node_nb in self.get_neighbor(node_cur):
                        if node_nb not in self.CloseList:
                            nb_cost = self.CostDict[node_cur] + self.get_cost(node_cur, node_nb)
                            if node_nb in self.CostDict:
                                if nb_cost < self.CostDict[node_nb]:
                                    self.ParentDict[node_nb] = node_cur
                                    self.CostDict[node_nb] = nb_cost
                                    heapq.heappush(self.OpenList, (nb_cost, node_nb))
                            else:
                                self.ParentDict[node_nb] = node_cur
                                self.CostDict[node_nb] = nb_cost
                                heapq.heappush(self.OpenList, (nb_cost, node_nb))

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
        self.VisitList = self.CloseList

def main():
    s_start = (5, 5)
    s_goal = (45, 5)

    dijk = Dijkstra(s_start, s_goal, 'Euclidean')
    dijk.search_path()
    dijk.get_path()
    dijk.get_visit()

    if dijk.PathList[0] == dijk.end_node:
        print("Path found!")
        print(f"Path length: ", len(dijk.PathList))
        print(f"Visited nodes: ", len(dijk.VisitList))
        print(f"Cost:", dijk.CostDict[dijk.PathList[0]])
    else:
        print("Path not found!")
    
    plot = plotting.Plotting(s_start, s_goal)
    plot.animation(dijk.PathList, dijk.VisitList, "Dijkstra Searching")  # animation


if __name__ == '__main__':
    main()