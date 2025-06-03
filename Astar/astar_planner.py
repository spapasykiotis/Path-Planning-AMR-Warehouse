"""
A_Star 2D

Original Author: zhm-real
Source: https://github.com/zhm-real/PathPlanning

Modified by: Spyros Papasykiotis

Description:
Added the ability to gather statistics
"""

import math
import heapq
import matplotlib.pyplot as plt
import plotting, env
import time

show_plot = True
show_visited = False
show_stats = True
show_path = False

class AStar:
    def __init__(self, s_start, s_goal, heuristic_type):
        self.s_start = s_start
        self.s_goal = s_goal
        self.heuristic_type = heuristic_type

        self.Env = env.Env()  # class Env
        self.Plot = plotting.Plotting(self.s_start, self.s_goal)

        self.u_set = self.Env.motions  # feasible input set
        self.obs = self.Env.obs  # position of obstacles
        
        self.OPEN = []  # priority queue / OPEN set
        self.CLOSED = []  # CLOSED set / VISITED order
        self.PARENT = dict()  # recorded parent
        self.g = dict()  # cost to come
        self.path = []
        
        self.visited = []
        self.total_visited = 0
        self.path_length = 0
        self.execution_time = 0.0

    def searching(self):
        start_time = time.time()
        
        self.PARENT[self.s_start] = self.s_start
        self.g[self.s_start] = 0
        self.g[self.s_goal] = math.inf
        heapq.heappush(self.OPEN,
                       (self.f_value(self.s_start), self.s_start))

        while self.OPEN:
            _, s = heapq.heappop(self.OPEN)
            self.CLOSED.append(s)

            if s == self.s_goal:  # stop condition
                break

            for s_n in self.get_neighbor(s):
                new_cost = self.g[s] + self.cost(s, s_n)

                if s_n not in self.g:
                    self.g[s_n] = math.inf

                if new_cost < self.g[s_n]:  # conditions for updating Cost
                    self.g[s_n] = new_cost
                    self.PARENT[s_n] = s
                    heapq.heappush(self.OPEN, (self.f_value(s_n), s_n))
        
        # Extract path
        path, visited = self.extract_path(self.PARENT), self.CLOSED
        self.path = path[::-1]
        self.visited = visited
        
        
        # Calculate statistics
        self.path_length = round(self.calculate_path_length(), 3)
        self.total_visited += len(visited)
        
        end_time = time.time()
        execution_time = end_time - start_time
        self.execution_time += round(execution_time, 5)
        
        return

    def get_neighbor(self, s):
        """
        find neighbors of state s that not in obstacles.
        :param s: state
        :return: neighbors
        """

        return [(s[0] + u[0], s[1] + u[1]) for u in self.u_set]

    def cost(self, s_start, s_goal):
        """
        Calculate Cost for this motion
        :param s_start: starting node
        :param s_goal: end node
        :return:  Cost for this motion
        :note: Cost function could be more complicate!
        """

        if self.is_collision(s_start, s_goal):
            return math.inf

        return math.hypot(s_goal[0] - s_start[0], s_goal[1] - s_start[1])

    def is_collision(self, s_start, s_end):
        """
        check if the line segment (s_start, s_end) is collision.
        :param s_start: start node
        :param s_end: end node
        :return: True: is collision / False: not collision
        """

        if s_start in self.obs or s_end in self.obs:
            return True

        if s_start[0] != s_end[0] and s_start[1] != s_end[1]:
            if s_end[0] - s_start[0] == s_start[1] - s_end[1]:
                s1 = (min(s_start[0], s_end[0]), min(s_start[1], s_end[1]))
                s2 = (max(s_start[0], s_end[0]), max(s_start[1], s_end[1]))
            else:
                s1 = (min(s_start[0], s_end[0]), max(s_start[1], s_end[1]))
                s2 = (max(s_start[0], s_end[0]), min(s_start[1], s_end[1]))

            if s1 in self.obs or s2 in self.obs:
                return True

        return False

    def f_value(self, s):
        """
        f = g + h. (g: Cost to come, h: heuristic value)
        :param s: current state
        :return: f
        """

        return self.g[s] + self.heuristic(s)

    def extract_path(self, PARENT):
        """
        Extract the path based on the PARENT set.
        :return: The planning path
        """

        path = [self.s_goal]
        s = self.s_goal

        while True:
            s = PARENT[s]
            path.append(s)

            if s == self.s_start:
                break

        return list(path)

    def heuristic(self, s):
        """
        Calculate heuristic.
        :param s: current node (state)
        :return: heuristic function value
        """

        heuristic_type = self.heuristic_type  # heuristic type
        goal = self.s_goal  # goal node

        if heuristic_type == "manhattan":
            return abs(goal[0] - s[0]) + abs(goal[1] - s[1])
        else:
            return math.hypot(goal[0] - s[0], goal[1] - s[1])

    def calculate_path_length(self):
        # Calculate the length of the generated path
        return sum(math.hypot(self.path[i][0] - self.path[i + 1][0], self.path[i][1] - self.path[i + 1][1])
            for i in range(len(self.path) - 1))
    
def show_statistics(instance):
    print(f"Total nodes expanded: {instance.total_visited}")
    print(f"Path Length: {instance.path_length}")
    print(f"Planning Time: {instance.execution_time} seconds")

def main():
    s_start = (5, 5)
    s_goal = (95, 95)

    astar = AStar(s_start, s_goal, "euclidean")
    astar.searching()
    
    if show_plot:
        astar.Plot.plot_grid_test("A*", plt.gca())
        if show_visited:
            astar.Plot.plot_visited(astar.visited)
        astar.Plot.plot_path(astar.path, lb="Calculated Path")

    if show_stats:
        show_statistics(astar)
    if show_path:
        # print("Path Lenght: ", len(path))
        for x, y in astar.path:
            print(f'{x, y}')
    if show_plot:
        plt.show()

if __name__ == '__main__':
    main()
