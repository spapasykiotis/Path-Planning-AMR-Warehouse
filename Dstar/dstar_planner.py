"""
D_Star 2D

Original Author: zhm-real
Source: https://github.com/zhm-real/PathPlanning

Modified by: Spyros Papasykiotis
"""

import math
import matplotlib.pyplot as plt
import time
import plotting
import env

show_plot = True
dynamic_obstacle = False
show_path = False
show_debug = False
show_visited = False
show_stats = True
show_legend = False


class DStar:
    def __init__(self, s_start, s_goal):
        self.s_start, self.s_goal = s_start, s_goal

        self.Env = env.Env()
        self.Plot = plotting.Plotting(self.s_start, self.s_goal)
        self.fig = None
    
        self.u_set = self.Env.motions
        self.obs = self.Env.obs
        self.x = self.Env.x_range
        self.y = self.Env.y_range

        self.OPEN = set()
        self.t = dict()
        self.PARENT = dict()
        self.h = dict()
        self.k = dict()
        self.path = []
        self.visited = set()
        self.count = 0
             
        self.execution_time = 0.0
        self.original_path_length = 0
        self.path_length = 0
        self.original_expanded_nodes = 0
        self.expanded_nodes = 0

    def init(self):
        for i in range(self.Env.x_range):
            for j in range(self.Env.y_range):
                self.t[(i, j)] = 'NEW'
                self.k[(i, j)] = 0.0
                self.h[(i, j)] = float("inf")
                self.PARENT[(i, j)] = None

        self.h[self.s_goal] = 0.0

    def run(self, s_start, s_end):
        start_time = time.time()  # Start time

        self.init()
        self.insert(s_end, 0)

        while True:
            self.process_state()
            if self.t[s_start] == 'CLOSED':
                print("Path found successfully.")
                break
        
        # Calculate statistics
        self.path = self.extract_path(s_start, s_end)
        self.original_path_length = len(self.path)
        self.path_length = round(self.calculate_path_length(), 3)
        
        self.original_expanded_nodes = len(self.visited)
        self.expanded_nodes = len(self.visited)
        
        end_time = time.time()                      # End time
        self.execution_time = end_time - start_time  # Calculate planning time
            
        # Calculate final stats
        self.execution_time = round(self.execution_time, 5)                       # round time value (3 decimal digits)
        
    def calculate_new_path(self, new_obs):
                          
        start_time = time.time() # Start counting recalculation time
            
        if show_debug:
            for (x, y) in new_obs:
                    print("Obstacle added at: s =", x, ",", "y =", y,)
        print()
                        
        
        # Add each new obstacle coordinate individually to self.obs
        for obs in new_obs:
            self.obs.add(obs)
        
        # Update plot obstacles
        self.Plot.update_obs(self.obs)

        s = self.s_start
        self.visited = set()
        self.count += 1

        while s != self.s_goal:
            if self.is_collision(s, self.PARENT[s]):
                self.modify(s)
                continue
            s = self.PARENT[s]

        # Update path and path length
        self.path = self.extract_path(self.s_start, self.s_goal)
        self.path_length = round(self.calculate_path_length(), 3)

        # End recalculation timer
        end_time = time.time()      

        # Update stats
        self.expanded_nodes += len(self.visited)
        self.execution_time += round(end_time - start_time, 5)  
            
    def extract_path(self, s_start, s_end):
        path = [s_start]
        s = s_start
        while True:
            s = self.PARENT[s]
            path.append(s)
            if s == s_end:
                return path

    def process_state(self):
        s = self.min_state()  # get node in OPEN set with min k value
        self.visited.add(s)

        if s is None:
            return -1  # OPEN set is empty

        k_old = self.get_k_min()  # record the min k value of this iteration (min path cost)
        self.delete(s)  # move state s from OPEN set to CLOSED set

        # k_min < h[s] --> s: RAISE state (increased cost)
        if k_old < self.h[s]:
            for s_n in self.get_neighbor(s):
                if self.h[s_n] <= k_old and \
                        self.h[s] > self.h[s_n] + self.cost(s_n, s):

                    # update h_value and choose parent
                    self.PARENT[s] = s_n
                    self.h[s] = self.h[s_n] + self.cost(s_n, s)

        # s: k_min >= h[s] -- > s: LOWER state (cost reductions)
        if k_old == self.h[s]:
            for s_n in self.get_neighbor(s):
                if self.t[s_n] == 'NEW' or \
                        (self.PARENT[s_n] == s and self.h[s_n] != self.h[s] + self.cost(s, s_n)) or \
                        (self.PARENT[s_n] != s and self.h[s_n] > self.h[s] + self.cost(s, s_n)):

                    # Condition:
                    # 1) t[s_n] == 'NEW': not visited
                    # 2) s_n's parent: cost reduction
                    # 3) s_n find a better parent
                    self.PARENT[s_n] = s
                    self.insert(s_n, self.h[s] + self.cost(s, s_n))
        else:
            for s_n in self.get_neighbor(s):
                if self.t[s_n] == 'NEW' or \
                        (self.PARENT[s_n] == s and self.h[s_n] != self.h[s] + self.cost(s, s_n)):

                    # Condition:
                    # 1) t[s_n] == 'NEW': not visited
                    # 2) s_n's parent: cost reduction
                    self.PARENT[s_n] = s
                    self.insert(s_n, self.h[s] + self.cost(s, s_n))
                else:
                    if self.PARENT[s_n] != s and \
                            self.h[s_n] > self.h[s] + self.cost(s, s_n):

                        # Condition: LOWER happened in OPEN set (s), s should be explored again
                        self.insert(s, self.h[s])
                    else:
                        if self.PARENT[s_n] != s and \
                                self.h[s] > self.h[s_n] + self.cost(s_n, s) and \
                                self.t[s_n] == 'CLOSED' and \
                                self.h[s_n] > k_old:

                            # Condition: LOWER happened in CLOSED set (s_n), s_n should be explored again
                            self.insert(s_n, self.h[s_n])

        return self.get_k_min()

    def min_state(self):
        """
        choose the node with the minimum k value in OPEN set.
        :return: state
        """

        if not self.OPEN:
            return None

        return min(self.OPEN, key=lambda x: self.k[x])

    def get_k_min(self):
        """
        calc the min k value for nodes in OPEN set.
        :return: k value
        """

        if not self.OPEN:
            return -1

        return min([self.k[x] for x in self.OPEN])

    def insert(self, s, h_new):
        """
        insert node into OPEN set.
        :param s: node
        :param h_new: new or better cost to come value
        """

        if self.t[s] == 'NEW':
            self.k[s] = h_new
        elif self.t[s] == 'OPEN':
            self.k[s] = min(self.k[s], h_new)
        elif self.t[s] == 'CLOSED':
            self.k[s] = min(self.h[s], h_new)

        self.h[s] = h_new
        self.t[s] = 'OPEN'
        self.OPEN.add(s)

    def delete(self, s):
        """
        delete: move state s from OPEN set to CLOSED set.
        :param s: state should be deleted
        """

        if self.t[s] == 'OPEN':
            self.t[s] = 'CLOSED'

        self.OPEN.remove(s)

    def modify(self, s):
        """
        start processing from state s.
        :param s: is a node whose status is RAISE or LOWER.
        """

        self.modify_cost(s)

        while True:
            k_min = self.process_state()

            if k_min >= self.h[s]:
                break

    def modify_cost(self, s):
        # if node in CLOSED set, put it into OPEN set.
        # Since cost may be changed between s - s.parent, calc cost(s, s.p) again

        if self.t[s] == 'CLOSED':
            self.insert(s, self.h[self.PARENT[s]] + self.cost(s, self.PARENT[s]))

    def get_neighbor(self, s):
        nei_list = set()

        for u in self.u_set:
            s_next = tuple([s[i] + u[i] for i in range(2)])
            if s_next not in self.obs:
                nei_list.add(s_next)

        return nei_list

    def cost(self, s_start, s_goal):
        """
        Calculate Cost for this motion
        :param s_start: starting node
        :param s_goal: end node
        :return:  Cost for this motion
        :note: Cost function could be more complicate!
        """

        if self.is_collision(s_start, s_goal):
            return float("inf")

        return math.hypot(s_goal[0] - s_start[0], s_goal[1] - s_start[1])

    def is_collision(self, s_start, s_end):
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

    def plot_visited(self, visited):
        color = ['gainsboro', 'lightgray', 'silver', 'darkgray',
                 'bisque', 'navajowhite', 'moccasin', 'wheat',
                 'powderblue', 'skyblue', 'lightskyblue', 'cornflowerblue']

        if self.count >= len(color) - 1:
            self.count = 0

        for x in visited:
            plt.plot(x[0], x[1], marker='s', color=color[self.count])
    
    def add_dynamic_obstacles(self, obs_n):
        
        new_obs = set()
        
        if  obs_n>0: 
            while(obs_n>0):
                    print("Adding new obstacle manually")
                    x = int(input("Enter x coordinate: "))
                    y = int(input("Enter y coordinate: "))
                    print()
                    if x < 0 or x > self.x - 1 or y < 0 or y > self.y - 1:
                            print("Please choose right area!")
                    else:
                        if (x, y) not in self.obs:      
                            new_obs.add((x,y))
                        else:
                            print(f'Obstacle {x,y} is already added')  
                        obs_n -= 1

        return new_obs
    
    def calculate_path_length(self):
        # Calculate the length of the generated path
        return sum(math.hypot(self.path[i][0] - self.path[i + 1][0], self.path[i][1] - self.path[i + 1][1])
            for i in range(len(self.path) - 1))
          
def show_stats(instance):
    # Print statistics method
    if instance.expanded_nodes != instance.original_expanded_nodes:
        print("Original nodes expanded:", instance.original_expanded_nodes)
        print("Original Path Length:", instance.original_path_nodes)
    print("Total nodes expanded:", instance.expanded_nodes)
    print("Final Path Length:", instance.path_length)
    print("Planning time:", instance.execution_time*1000, "ms")

def main():
    s_start = (5, 5)
    s_goal = (95, 95)
    dstar = DStar(s_start, s_goal)
    dstar.run(s_start, s_goal)

    # Plotting path using plotting method and plotting library
    dstar.fig = plt.figure()
    dstar.Plot.plot_grid_test("Dynamic A* (D*)",plt.gca())
    dstar.Plot.plot_path(dstar.path, lb = f'Original Path')

    # Adding n number of dynamic obstacles
    if dynamic_obstacle:   
        obstacles_number = int(input("Obstacles number: "))
        print()
        if obstacles_number > 0: 
            # Check new obstacles and add to a set  
            new_obs = dstar.add_dynamic_obstacles(obstacles_number)
            # Calculate new path using valid new obstacles
            dstar.calculate_new_path(new_obs)
            # Update plot with new path
            plt.cla()
            dstar.Plot.plot_grid_test("Dynamic A* Lite (D* Lite)",plt.gca())
            if show_visited:
                dstar.plot_visited(dstar.visited)
            dstar.Plot.plot_path(dstar.path, lb = f'Updated Path')
        
    # Access statistics
    if show_stats:
        show_stats(dstar)
        print()
        
    if show_path:
        for x, y in dstar.path:
            print(f'{x, y}')
    
    if show_plot:
            if show_legend:
                plt.legend(loc='upper right')
            plt.show()

if __name__ == '__main__':
    main()
