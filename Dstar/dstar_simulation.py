"""
D_star 2D multirobot simulation
@author: Spyros Papasykiotis
"""

import matplotlib.pyplot as plt
import dstar_planner as d_star
import math
import random
import pandas as pd

show_plot = True
show_stats = True
show_grid = False
show_legend = False
time_simulation = True
show_visited = False
show_debug = False

class SimulatedRobot:
    
    def __init__(self, start, goal, id):
        self.start = start
        self.goal = goal
        self.path_recalculations = 0
        self.id = id

    def add_obstacle(self, x, y):
        # Add an obstacle covering only the occupied node
        obstacle = {'x': x, 'y': y, 'width': 0, 'height': 0}
        self.obstacles.append(obstacle)


def add_rectangular_obstacle(ox, oy, x, y, width, height, step=1):
    
    x, y, width, height = int(x), int(y), int(width), int(height)

    for i in range(x, x + width, step):
        ox.append(i)
        oy.append(y)
    for i in range(y, y + height, step):
        ox.append(x + width)
        oy.append(i)
    for i in range(x, x + width + step, step):
        ox.append(i)
        oy.append(y + height)
    for i in range(y, y + height + step, step):
        ox.append(x)
        oy.append(i)


def add_obstacles(ox, oy, obstacles):
    for obstacle in obstacles:
        add_rectangular_obstacle(ox, oy, **obstacle)

def current_iteration_position(current_coord, iteration):
    current_pos = current_coord[min(iteration, len(current_coord)-1) if iteration >= 0 else 0]
    return current_pos

def validate_path(x_coordinates, y_coordinates):
    
    nodes = list(zip(x_coordinates, y_coordinates))
    unique_nodes = set()
    non_unique_nodes = set()

    if len(set(nodes)) == len(nodes):
        return True
    
    for node in nodes:
        if node in unique_nodes:
            non_unique_nodes.add(node)
        else:
            unique_nodes.add(node)  

    return list(non_unique_nodes)

def find_non_unique_nodes(x_coordinates, y_coordinates):
    nodes = list(zip(x_coordinates, y_coordinates))
    unique_nodes = set()
    non_unique_nodes = set()

    for node in nodes:
        if node in unique_nodes:
            non_unique_nodes.add(node)
        else:
            unique_nodes.add(node)

    return list(non_unique_nodes)
  
def is_nearby(robot1, robot2, radius=1):
    x1, y1 = robot1
    x2, y2 = robot2
    distance_squared = (x1 - x2) ** 2 + (y1 - y2) ** 2
    radius_squared = radius ** 2
    return distance_squared <= radius_squared  
  
def check_robot_positions(all_robot_data, iteration):
    
    # Iterate through each unique pair of robots
    for i, (planner_data, _) in enumerate(all_robot_data):
        for j, (other_planner_data, _) in enumerate(all_robot_data):
            # Skip checking with itself
            if i == j:
                continue
            # Determine the positions to compare based on the current iteration
            current_x = planner_data.path[min(iteration, len(planner_data.path)-1) if iteration >= 0 else 0][0]
            current_y = planner_data.path[min(iteration, len(planner_data.path)-1) if iteration >= 0 else 0][1]
            other_x = other_planner_data.path[min(iteration, len(other_planner_data.path)-1) if iteration >= 0 else 0][0]
            other_y = other_planner_data.path[min(iteration, len(other_planner_data.path)-1) if iteration >= 0 else 0][1]
 
            # print(planner_data.path[i][1]) #DEBUG
            
            robot1 = (current_x, current_y)
            robot2 = (other_x, other_y)
            
            # Compare positions and recalculate path if second robot is in first robot radius
            if is_nearby(robot1,robot2, radius=0.5):
                print(f"Collision detected at node {current_x, current_y} ! Robot {i + 1} and Robot {j + 1} at the same node in iteration {iteration} \n")
                plt.plot(current_x, current_y, 'ko', markersize=5, label=f'R{i + 1} and R{j + 1} Collision Point')
                
                # Check which robot is closer to its goal to recalculate path
                nodes_remaining_i = len(planner_data.path) - iteration
                nodes_remaining_j = len(other_planner_data.path) - iteration
                
                if show_debug:
                    print(f"Nodes remaining for Robot {i+1}: \n", nodes_remaining_i)
                    print(f"Nodes remaining for Robot {j+1}: \n", nodes_remaining_j)
                
                if nodes_remaining_i  <=0 and nodes_remaining_j <= 0:
                    print(f"!Warning!: Robots {i + 1} and {j + 1} have reached their goals and are at the same node in iteration {iteration} \n")              
                else:                    
                    if nodes_remaining_i  > 0 and nodes_remaining_j > 0 :
                        if nodes_remaining_i < nodes_remaining_j:
                            closer_robot = i
                        elif nodes_remaining_i > nodes_remaining_j:
                            closer_robot = j
                        else:
                            # If they have the same nodes remaining, choose randomly
                            closer_robot = random.choice([i, j])      
                        
                    elif nodes_remaining_i > 0:
                        closer_robot = i
                    elif nodes_remaining_j > 0:
                        closer_robot = j
                    
                    # Call the recalculate_path method
                    print(f"Recalculating path for Robot {closer_robot + 1}")
                    print() 
                    new_path = recalculate_path(all_robot_data, closer_robot, iteration)
                    
                    # Plot updated path only
                    if show_visited:
                        all_robot_data[closer_robot][0].plot_visited(all_robot_data[closer_robot][0].visited)
                    all_robot_data[closer_robot][0].Plot.plot_path(new_path[iteration-2:], line='--', lb=f'Robot {closer_robot + 1} Updated Path')

    return all_robot_data

def recalculate_path(all_robot_data, closer_robot, iteration):
     
    robot_positions = set()

    # Add an obstacle representing the current position of every other robot
    for check_robot in all_robot_data:
        if check_robot[1].id != all_robot_data[closer_robot][1].id:  # Exclude the current robot itself
            current_pos = current_iteration_position(check_robot[0].path, iteration) # Get current position as (x,y) tupple
            if show_debug:                   
                print(f'Robot:{check_robot[1].id} is at {current_pos} in iterarion {iteration}')
            robot_positions.add(current_pos)
            
    # Recalculate path by using the current robot position as obstacles
    all_robot_data[closer_robot][0].calculate_new_path(robot_positions)
    
    # Save number of recalculations for each robot
    all_robot_data[closer_robot][1].path_recalculations +=1

    # Return the new path and the updated robot
    return all_robot_data[closer_robot][0].path

def print_robot_statistics_to_excel(final_robot_data, output_file):
    # Create lists to store data
    robot_numbers = []
    total_visited_nodes = []
    path_lengths = []
    execution_times = []
    path_recalculations = []

    # Extract data from final_robot_data
    for i, (planner_data, robot_data) in enumerate(final_robot_data):
        robot_numbers.append(i + 1)
        total_visited_nodes.append(planner_data.expanded_nodes)
        path_lengths.append(planner_data.path_length)
        execution_times.append(planner_data.execution_time*1000)
        path_recalculations.append(robot_data.path_recalculations)

    # Create a DataFrame
    df = pd.DataFrame({
        'Robot': robot_numbers,
        'Total Visited Nodes': total_visited_nodes,
        'Path Length': path_lengths,
        'Execution Time (s)': execution_times,
        'Path Recalculations': path_recalculations
    })

    # Write DataFrame to Excel file
    df.to_excel(output_file, index=False)

def main():
    print(__file__ + " start!!")
    
    # create robots
    robots = [
        SimulatedRobot(start=(40.0, 5.0), goal=(40.0, 95.0), id=1),
        SimulatedRobot(start=(60.0, 5.0), goal=(60.0, 95.0), id=2),
        SimulatedRobot(start=(5.0, 40), goal=(95.0, 40.0), id=3),
        SimulatedRobot(start=(5.0, 60), goal=(95.0, 60.0), id=4),
        SimulatedRobot(start=(5.0, 5.0), goal=(95.0, 95.0), id=5),
        SimulatedRobot(start=(95.0, 5.0), goal=(5.0, 95.0), id=6),
        # Collision testing robot:
        # SimulatedRobot(start=(80.0, 70.0), goal=(91.0, 70.0), id=99)
    ]

    max_nodes = 0
    all_robot_data = []

    if show_plot:
        plt.figure()

    # Calculate paths once and find the maximum number of nodes in the final path among all robots
    for i, robot_data in enumerate(robots):

        start = robot_data.start
        goal = robot_data.goal
        dstar = d_star.DStar(start, goal)
        dstar.run(start, goal)

        all_robot_data.append((dstar,  robot_data))    # Add planner data to global robot variable    
        max_nodes = max(max_nodes, len(dstar.path))  # Update num_iterations max_nodes_in_final_path
        
        if show_plot:  
            dstar.Plot.plot_path(dstar.path, lb=f'Robot {i + 1} Path')
        
        if show_stats:
            print(f'Robot {i + 1} Statistics:')
            d_star.show_stats(dstar)
            print()       
            
    # Plotting grid once (every dstar instance has same env)
    dstar.Plot.plot_grid_test("Dynamic A*", plt.gca()) 
    
    # print(all_robot_paths[0]) #DEBUG
    # print("Max nodes (iteration steps):", max_nodes) #DEBUG
        
    # Time Simulation
    if time_simulation:
        for iteration in range(max_nodes):
            final_robot_data = check_robot_positions(all_robot_data, iteration)

        # Print robot final statistics
        if show_stats:
            print("All robot Statistics\n")
            for i, (planner_data, robot_data) in enumerate(final_robot_data):
                print(f"Robot {i + 1}: ")
                print(f'  Total nodes visited nodes: {planner_data.expanded_nodes} nodes')
                print(f'  Path length: {planner_data.path_length} nodes')
                print(f"  Execution time: {planner_data.execution_time*1000} ms")
                print(f"  Path recalculations: {robot_data.path_recalculations}")
            print()
            print_robot_statistics_to_excel(final_robot_data, 'robot_statistics.xlsx')
        
    if show_plot:
        if show_legend:
            plt.legend(loc='upper right')
        
    if show_plot:
        plt.show()


if __name__ == '__main__':
    main()
