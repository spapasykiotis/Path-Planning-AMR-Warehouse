"""
D_Star_Lite 2D

Original Author: zhm-real
Source: https://github.com/zhm-real/PathPlanning

Modified by: Spyros Papasykiotis
"""

import math
import time
import matplotlib.pyplot as plt
import plotting, env

show_plot = True
dynamic_obstacle = False
show_path = False
show_debug = False
show_visited = False
show_stats = True
show_legend = False

class DStar:
    def __init__(self, s_start, s_goal, heuristic_type):
        self.s_start, self.s_goal = s_start, s_goal
        self.heuristic_type = heuristic_type

        self.Env = env.Env()  # class Env
        self.Plot = plotting.Plotting(s_start, s_goal)
        self.fig = None
        

        self.u_set = self.Env.motions  # feasible input set
        self.obs = self.Env.obs  # position of obstacles
        self.x = self.Env.x_range
        self.y = self.Env.y_range

        self.g, self.rhs, self.U = {}, {}, {}
        self.km = 0

        for i in range(1, self.Env.x_range - 1):
            for j in range(1, self.Env.y_range - 1):
                self.rhs[(i, j)] = float("inf")
                self.g[(i, j)] = float("inf")

        self.rhs[self.s_goal] = 0.0
        self.U[self.s_goal] = self.CalculateKey(self.s_goal)
        self.visited = set()
        self.count = 0
        self.path = []
        
        self.original_path_length = 0
        self.original_expanded_nodes = 0
        self.path_length = 0
        self.expanded_nodes = 0
        self.execution_time = 0.0

    def run(self):
        
        start_time = time.time()  # Start time
        
        # Compute path to goal
        self.ComputePath()
        
        # Extracting path and path length
        self.path = self.extract_path()
        self.original_path_length = round(self.calculate_path_length(),3)
        self.path_length = round(self.calculate_path_length(),3)
        
        # Calculate statistics
        self.original_expanded_nodes = len(self.visited)
        self.expanded_nodes = len(self.visited)
        
        # Calculate time
        end_time = time.time()                              # End time
        self.execution_time = end_time - start_time          # Calculate planning time
        self.execution_time = round(self.execution_time, 5)   # round time value

    def calculate_new_path(self, new_obs):

        start_time = time.time()  # Start counting recalculation time

        s_curr = self.s_start
        s_last = self.s_start
        i = 0
        path = [self.s_start]

        if show_debug:
            for (x, y) in new_obs:
                    print("Obstacle added at: s =", x, ",", "y =", y,)
        print()

        while s_curr != self.s_goal:
            s_list = {}

            for s in self.get_neighbor(s_curr):
                s_list[s] = self.g[s] + self.cost(s_curr, s)
            s_curr = min(s_list, key=s_list.get)
            path.append(s_curr)

            if i < 1:
                self.km += self.h(s_last, s_curr)
                s_last = s_curr

                # Add each new obstacle coordinate individually to self.obs
                for obs in new_obs:
                    self.obs.add(obs)

                # Update g and rhs dictionaries appropriately
                for obs in new_obs:
                    self.g[obs] = float("inf")
                    self.rhs[obs] = float("inf")

                # Update vertices for each neighbor of the current obstacle
                for obs in new_obs:
                    for s in self.get_neighbor(obs):
                        self.UpdateVertex(s)

                i += 1

                self.count += 1
                self.visited = set()
                self.ComputePath()

        if show_visited:
            self.plot_visited(self.visited)

        # Save new path and its length
        self.path = self.extract_path()
        self.path_length = round(self.calculate_path_length(),3)
        self.expanded_nodes += len(self.visited)

        # Update plot obstacles
        self.Plot.update_obs(self.obs)
        
        # End recalculation timer
        end_time = time.time()

        # Update stats
        self.execution_time += (end_time - start_time)
        self.execution_time = round(self.execution_time, 5)


    def ComputePath(self):
        
        while True:
            s, v = self.TopKey()
            if v >= self.CalculateKey(self.s_start) and \
                    self.rhs[self.s_start] == self.g[self.s_start]:
                break

            k_old = v
            self.U.pop(s)
            self.visited.add(s)

            if k_old < self.CalculateKey(s):
                self.U[s] = self.CalculateKey(s)
            elif self.g[s] > self.rhs[s]:
                self.g[s] = self.rhs[s]
                for x in self.get_neighbor(s):
                    self.UpdateVertex(x)
            else:
                self.g[s] = float("inf")
                self.UpdateVertex(s)
                for x in self.get_neighbor(s):
                    self.UpdateVertex(x)


    def UpdateVertex(self, s):
        if s != self.s_goal:
            self.rhs[s] = float("inf")
            for x in self.get_neighbor(s):
                self.rhs[s] = min(self.rhs[s], self.g[x] + self.cost(s, x))
        if s in self.U:
            self.U.pop(s)

        if self.g[s] != self.rhs[s]:
            self.U[s] = self.CalculateKey(s)

    def CalculateKey(self, s):
        return [min(self.g[s], self.rhs[s]) + self.h(self.s_start, s) + self.km,
                min(self.g[s], self.rhs[s])]

    def TopKey(self):
        """
        :return: return the min key and its value.
        """

        s = min(self.U, key=self.U.get)
        return s, self.U[s]

    def h(self, s_start, s_goal):
        heuristic_type = self.heuristic_type  # heuristic type

        if heuristic_type == "manhattan":
            return abs(s_goal[0] - s_start[0]) + abs(s_goal[1] - s_start[1])
        else:
            return math.hypot(s_goal[0] - s_start[0], s_goal[1] - s_start[1])

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

    def get_neighbor(self, s):
        nei_list = set()
        for u in self.u_set:
            s_next = tuple([s[i] + u[i] for i in range(2)])
            if s_next not in self.obs:
                nei_list.add(s_next)

        return nei_list

    def extract_path(self):
        """
        Extract the path based on the PARENT set.
        :return: The planning path
        """

        path = [self.s_start]
        s = self.s_start

        while s != self.s_goal:
            g_list = {}
            for x in self.get_neighbor(s):
                if not self.is_collision(s, x):
                    g_list[x] = self.g[x]

            if not g_list:
                print("No valid neighbors found. Failed to find path.")
                return None  # Return None if no valid neighbors found

            s = min(g_list, key=g_list.get)
            path.append(s)
            
        if show_debug:   
            print("Path found successfully.")
            
        return list(path)

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
      
def show_statistics(instance):
    # Print statistics method
    if instance.expanded_nodes != instance.original_expanded_nodes:
        print("Original nodes expanded:", instance.original_expanded_nodes)
        print("Original nodes in final path:", instance.original_path_length)
    print("Total nodes expanded:", instance.expanded_nodes)
    print("Final path length:", instance.path_length)
    print("Planning time:", instance.execution_time*1000, "ms")

def main():
    
    start = (5, 5)
    goal = (95, 95)
    dstar = DStar(start, goal, "euclidean")
    dstar.run()

    # Plotting path using plotting method and plotting library
    dstar.Plot.plot_grid_test("D* Lite",plt.gca())
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
            dstar.Plot.plot_grid_test("D* Lite",plt.gca())
            if show_visited:
                dstar.plot_visited(dstar.visited)
            dstar.Plot.plot_path(dstar.path, lb = f'Updated Path')

    # Access statistics    
    if show_stats:
        show_statistics(dstar)
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
