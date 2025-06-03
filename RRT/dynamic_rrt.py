"""
Dynamic_RRT 2D

Original Author: zhm-real
Source: https://github.com/zhm-real/PathPlanning

Modified by: Spyros Papasykiotis

Description:
Modified to work with the grid world approach.
"""

import math
import copy
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import time
import env, plotting, utils

show_plot = True
show_stats = True
show_visited = False
show_drrt_path = False
show_movement_path = False
verify_movement_path = False
show_debug = False
show_legend = False
dynamic_obstacle = True


class Node:
    def __init__(self, n):
        self.x = n[0]
        self.y = n[1]
        self.parent = None
        self.flag = "VALID"


class Edge:
    def __init__(self, n_p, n_c):
        self.parent = n_p
        self.child = n_c
        self.flag = "VALID"


class DynamicRrt:
    def __init__(self, s_start, s_goal, step_len, goal_sample_rate, waypoint_sample_rate, iter_max):
        self.s_start = Node(s_start)
        self.s_goal = Node(s_goal)
        self.step_len = step_len
        self.goal_sample_rate = goal_sample_rate
        self.waypoint_sample_rate = waypoint_sample_rate
        self.iter_max = iter_max
        self.vertex = [self.s_start]
        self.vertex_old = []
        self.vertex_new = []
        self.edges = []

        self.env = env.Env()
        self.plotting = plotting.Plotting(s_start, s_goal)
        self.utils = utils.Utils()
        

        self.x_range = self.env.x_range
        self.y_range = self.env.y_range
        self.obs_rectangle = self.env.obs_rectangle
        self.obs_boundary = self.env.obs_boundary
        self.obs_add = []  # Initialize obs_add as an empty list

        self.path = []
        self.waypoint = []
        self.movement_path = []
        self.original_movement_path = []

        # Statistics tracking
        self.original_path_length = 0
        self.path_length = 0
        self.original_explored_nodes = 0
        self.explored_nodes = 0
        self.replanning_count = 0
        self.execution_time = 0.0

    def planning(self):
        start_time = time.time()

        for _ in range(self.iter_max):
            node_rand = self.generate_random_node(self.goal_sample_rate)
            node_near = self.nearest_neighbor(self.vertex, node_rand)
            node_new = self.new_state(node_near, node_rand)

            if node_new and not self.utils.is_collision(node_near, node_new):
                self.vertex.append(node_new)
                self.edges.append(Edge(node_near, node_new))
                dist, _ = self.get_distance_and_angle(node_new, self.s_goal)

                if dist <= self.step_len:
                    self.new_state(node_new, self.s_goal)
                    
                    print(f'Path Found')
                    self.path = self.extract_path(node_new)
                    self.original_path_length = round(self.calculate_path_length(), 3)
                    self.path_length = self.original_path_length
                    self.extract_movement_path()
                    self.original_movement_path = self.movement_path
                    self.original_explored_nodes = len(self.vertex)
                    self.explored_nodes = len(self.vertex)
                    
                    self.waypoint = self.extract_waypoint(node_new)
                    
                    end_time = time.time()
                    self.execution_time += round(end_time - start_time, 5)
                    return

        print(f'Path not found')
        end_time = time.time()
        self.execution_time += round(end_time - start_time, 3)

        return

    def calculate_new_path(self):

        if show_debug:
            for (x, y, width, height) in self.obs_add:
                print(f'Obstacle added at: x: {x}, y: {y}, width: {width}, height: {height}')
        print()

        self.utils.update_obs(self.obs_boundary, self.obs_rectangle)
        self.InvalidateNodes()

        if self.is_path_invalid():
            print("Replanning path!")
            path, waypoint = self.replanning()

            # print("len_vertex: ", len(self.vertex))
            # print("len_vertex_old: ", len(self.vertex_old))
            # print("len_vertex_new: ", len(self.vertex_new))

            self.vertex_new = []
            self.path = path
            self.path_length = round(self.calculate_path_length(), 3)
            self.extract_movement_path()
            self.replanning_count += 1
            self.explored_nodes += len(self.vertex)        
            
            self.waypoint = waypoint    
        else:
            print('No need for replanning')
            print('Trimming Invalid Nodes')
            self.TrimRRT()

    def InvalidateNodes(self):
        for edge in self.edges:
            if self.is_collision_obs_add(edge.parent, edge.child):
                edge.child.flag = "INVALID"

    def is_path_invalid(self):
        for node in self.waypoint:
            if node.flag == "INVALID":
                return True

    def is_collision_obs_add(self, start, end):
        delta = self.utils.delta
        obs_add = self.obs_add

        for obstacle in obs_add:
            x, y, width, height = obstacle
            if (x - delta) <= start.x <= (x + width + delta) and \
                    (y - delta) <= start.y <= (y + height + delta):
                return True

            if (x - delta) <= end.x <= (x + width + delta) and \
                    (y - delta) <= end.y <= (y + height + delta):
                return True

        return False

    def replanning(self):
        self.TrimRRT()

        start_time = time.time()

        for _ in range(self.iter_max):
            node_rand = self.generate_random_node_replanning(self.goal_sample_rate, self.waypoint_sample_rate)
            node_near = self.nearest_neighbor(self.vertex, node_rand)
            node_new = self.new_state(node_near, node_rand)

            if node_new and not self.utils.is_collision(node_near, node_new) and \
                    not self.is_collision_obs_add(node_near, node_new):  # Check collision with obstacles
                self.vertex.append(node_new)
                self.vertex_new.append(node_new)
                self.edges.append(Edge(node_near, node_new))
                dist, _ = self.get_distance_and_angle(node_new, self.s_goal)

                if dist <= self.step_len:
                    self.new_state(node_new, self.s_goal)
                    path = self.extract_path(node_new)
                    waypoint = self.extract_waypoint(node_new)
                    if show_debug:
                        print(f'Path found including: {len(path)} nodes') 
                        print(f'Waypoints: {len(waypoint)}')

                    end_time = time.time()
                    self.execution_time += round(end_time - start_time, 5)
                    
                    return path, waypoint
        print('Path not found ')

        end_time = time.time()
        self.execution_time += round(end_time - start_time, 3)

        return None

    def TrimRRT(self):    
        start_time = time.time()
        
        for i in range(1, len(self.vertex)):
            node = self.vertex[i]
            node_p = node.parent
            if node_p.flag == "INVALID":
                node.flag = "INVALID"

        self.vertex = [node for node in self.vertex if node.flag == "VALID"]
        self.vertex_old = copy.deepcopy(self.vertex)
        self.edges = [Edge(node.parent, node) for node in self.vertex[1:len(self.vertex)]]
        
        end_time = time.time()
        self.execution_time += round(end_time - start_time, 5)

    def generate_random_node(self, goal_sample_rate):
        delta = self.utils.delta

        if np.random.random() > goal_sample_rate:
            return Node((np.random.uniform(self.x_range[0] + delta, self.x_range[1] - delta),
                         np.random.uniform(self.y_range[0] + delta, self.y_range[1] - delta)))

        return self.s_goal

    def generate_random_node_replanning(self, goal_sample_rate, waypoint_sample_rate):
        delta = self.utils.delta
        p = np.random.random()

        if p < goal_sample_rate:
            return self.s_goal
        elif goal_sample_rate < p < goal_sample_rate + waypoint_sample_rate:
            return self.waypoint[np.random.randint(0, len(self.waypoint) - 1)]
        else:
            return Node((np.random.uniform(self.x_range[0] + delta, self.x_range[1] - delta),
                         np.random.uniform(self.y_range[0] + delta, self.y_range[1] - delta)))

    @staticmethod
    def nearest_neighbor(node_list, n):
        return node_list[int(np.argmin([math.hypot(nd.x - n.x, nd.y - n.y)
                                        for nd in node_list]))]

    def new_state(self, node_start, node_end):
        dist, theta = self.get_distance_and_angle(node_start, node_end)

        dist = min(self.step_len, dist)
        node_new = Node((node_start.x + dist * math.cos(theta),
                         node_start.y + dist * math.sin(theta)))
        node_new.parent = node_start

        return node_new

    def extract_path(self, node_end):
        path = [(self.s_goal.x, self.s_goal.y)]
        node_now = node_end

        while node_now.parent is not None:
            node_now = node_now.parent
            path.append((node_now.x, node_now.y))

        return path

    def extract_waypoint(self, node_end):
        waypoint = [self.s_goal]
        node_now = node_end

        while node_now.parent is not None:
            node_now = node_now.parent
            waypoint.append(node_now)

        return waypoint

    @staticmethod
    def get_distance_and_angle(node_start, node_end):
        dx = node_end.x - node_start.x
        dy = node_end.y - node_start.y
        return math.hypot(dx, dy), math.atan2(dy, dx)

    def plot_grid(self, name):

        for (ox, oy, w, h) in self.obs_boundary:
            self.ax.add_patch(
                patches.Rectangle(
                    (ox, oy), w, h,
                    edgecolor='black',
                    facecolor='black',
                    fill=True
                )
            )

        for (ox, oy, w, h) in self.obs_rectangle:
            self.ax.add_patch(
                patches.Rectangle(
                    (ox, oy), w, h,
                    edgecolor='black',
                    facecolor='gray',
                    fill=True
                )
            )

        plt.title(name)
        plt.axis("equal")

    def plot_visited(self, animation=True):
        if animation:
            count = 0
            for node in self.vertex:
                count += 1
                if node.parent:
                    plt.plot([node.parent.x, node.x], [node.parent.y, node.y], "-g")
                    plt.gcf().canvas.mpl_connect('key_release_event',
                                                 lambda event:
                                                 [exit(0) if event.key == 'escape' else None])
                    if count % 10 == 0:
                        plt.pause(0.001)
        else:
            for node in self.vertex:
                if node.parent:
                    plt.plot([node.parent.x, node.x], [node.parent.y, node.y], "-g")

    def plot_vertex_old(self):
        for node in self.vertex_old:
            if node.parent:
                plt.plot([node.parent.x, node.x], [node.parent.y, node.y], "-g")

    def plot_vertex_new(self):
        count = 0

        for node in self.vertex_new:
            count += 1
            if node.parent:
                plt.plot([node.parent.x, node.x], [node.parent.y, node.y], color='darkorange')
                plt.gcf().canvas.mpl_connect('key_release_event',
                                             lambda event:
                                             [exit(0) if event.key == 'escape' else None])
                if count % 10 == 0:
                    plt.pause(0.001)

    def add_dynamic_obstacles(self, obs_n):

        if obs_n > 0:
            while obs_n > 0:
                print("Adding new obstacle manually")
                x = int(input("Enter x coordinate: "))
                y = int(input("Enter y coordinate: "))
                width = int(input("Enter width: "))
                height = int(input("Enter height: "))
                print()
                if x < 0 or x > self.x_range[1] - 1 or y < 0 or y > self.y_range[1] - 1:
                    print("Please choose right area!")
                else:
                    if [x, y, width, height] not in self.obs_add:
                        self.obs_add.append([x, y, width, height])  # Append obstacle to obs_add
                        self.obs_rectangle.append([x, y, width, height])
                    else:
                        print(f'Obstacle {x, y} is already added')
                    obs_n -= 1
        return

    def calculate_path_length(self):
        # Calculate the length of the generated path
        return sum(math.hypot(self.path[i][0] - self.path[i + 1][0], self.path[i][1] - self.path[i + 1][1])
            for i in range(len(self.path) - 1))
  
    def show_statistics(self):
        print("Original nodes explored:", self.original_explored_nodes)
        print("Original DRRT path length:", self.original_path_length)
        print("Original movement path nodes:", len(self.original_movement_path))
        if self.replanning_count > 0:
            print("Path recalculations:", self.replanning_count)
            print("Total nodes explored:", self.explored_nodes) 
            print("DRRT path Length:", self.path_length)
            print("Movement path nodes:", len(self.movement_path))
        print("Execution time:", round(self.execution_time, 5))
    
    def extract_movement_path(self):
        path = self.path[::-1]
        movement_path = []
        # print(path)
        movement_path.append((round(path[0][0], 3), round(path[0][1], 3)))   # Add first point to movement path (rounded to 3 decimals)
        if len(path) > 0:
            for i in range(len(path) - 1):
                x1, y1 = path[i]
                x2, y2 = path[i + 1]
                movement_path += self.interpolate_points(x1, y1, x2, y2)
                self.movement_path = movement_path
        else:
            print("Cannot extract movement path, path length is ", len(path))     
        return

    def interpolate_points(self, x1, y1, x2, y2):
    
        dx = x2-x1
        dy = y2-y1
        distance = math.hypot(dx, dy)
        
        if distance < 1.5:
            return [(round(x2,3), round(y2,3))]
                
        # Calculate the number of spaces between points (evenly distributed)
        if distance - int(distance)<0.5:  
            spaces = int(distance)
        else:
            spaces = int(distance) +1
        
        # points = spaces -1
        
        # Calculate the increment value for evenly spacing the points
        increment = distance / spaces
        
        interpolated_points = []
        # Iterate over the line segment, adding interpolated points
        for i in range(1, spaces):
            # Calculate the coordinates of the interpolated point
            x = x1 + (x2 - x1) * (i * increment / distance)
            y = y1 + (y2 - y1) * (i * increment / distance)
            interpolated_points.append((round(x,3), round(y,3)))
        
        return interpolated_points + [(round(x2,3), round(y2,3))]

def euclidean_distance(point1, point2):
    return math.sqrt((point2[0] - point1[0])**2 + (point2[1] - point1[1])**2)

def verify_distance(extracted_path):
    distances = []
    for i in range(len(extracted_path) - 1):
        distance = euclidean_distance(extracted_path[i], extracted_path[i + 1])
        distances.append(round(distance, 3))
    return distances

def main():
    start = (5, 5)  # Starting node
    goal = (95, 95)  # Goal node

    drrt = DynamicRrt(start, goal, 4, 0.01, 0.5, 5000)  # start, goal, step length, goal sampling rate, waypoint sampling rate, maximum iterations
    drrt.planning()
    
    # Access statistics    
    if show_stats and drrt.path:
        drrt.show_statistics()
        print()

    if show_drrt_path and drrt.path:
        print(f'DRRT Path nodes: {len(drrt.path)}')
        for x, y in drrt.path:
            print(f'{x, y}')
        print()

    if show_plot:
        drrt.plotting.plot_grid("Dynamic RRT", plt.gca())
        plt.plot(start[0], start[1], "sg", linewidth=3)
        plt.plot(goal[0], goal[1], "sr", linewidth=3)

        if show_visited:
            drrt.plot_visited()
        drrt.plotting.plot_path(drrt.path,cl='red', lb="Original Path")
    if dynamic_obstacle:
        obstacles_number = int(input("Obstacles number: "))
        print()
        if obstacles_number > 0:
            drrt.add_dynamic_obstacles(obstacles_number)  # Add new obstacle
            drrt.calculate_new_path()  # Recalculate path
            drrt.plotting.update_obs(drrt.obs_boundary, drrt.obs_rectangle)  # Update Plotting obstacles

            # Update plot with new path
            plt.cla()

            drrt.plotting.plot_grid("Dynamic RRT",  plt.gca())
            if show_visited:
                drrt.plot_visited()
            drrt.plot_vertex_old()
            drrt.plot_vertex_new()
            drrt.plotting.plot_path(drrt.path,cl='purple', lb=f'Updated Path')
        # drrt.fig.canvas.mpl_connect('button_press_event', drrt.calculate_new_path)
        
        if show_stats and drrt.path:
            drrt.show_statistics()
            print()
    
    if show_movement_path and drrt.movement_path:
        print(f'Movement Path nodes: {len(drrt.movement_path)}')
        for x, y in drrt.movement_path:
            print(f'{x, y}')
        print()
        drrt.plotting.plot_path(drrt.movement_path,cl='blue',line='--', lb="Movement Path")
        if verify_movement_path:
            distances = verify_distance(drrt.movement_path) 
            # Print the distances
            print("Euclidean distances between adjacent points:")
            for i, distance in enumerate(distances):
                print(f"Distance {i+1}: {distance}")   

    
        
    if show_plot:
        if show_legend:
            plt.legend(loc='upper right')
        plt.show()

if __name__ == '__main__':
    main()
