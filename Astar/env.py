"""
Env 2D

Original Author: zhm-real
Source: https://github.com/zhm-real/PathPlanning

Modified by: Spyros Papasykiotis

Description:
Modified the environment to simulate a warehouse and added 4 warehouse presets.
"""

warehouse = 1

class Env:
    def __init__(self):
        self.x_range = 100  # size of background
        self.y_range = 100
        self.motions = [(-1, 0), (-1, 1), (0, 1), (1, 1),
                        (1, 0), (1, -1), (0, -1), (-1, -1)]
        self.obs = self.obs_map()
        self.obs_rect = []
        self.obs_bound = []

    def update_obs(self, obs):
        self.obs = obs
        
    def add_rectangular_obstacle(self, obs, x, y, width, height, step=1):
    
        if type(x) != int and type(y) != int and type(width) != int and type(height) != int:
            print("Warning: Casting arguments to integer")
    
        x, y, width, height = int(x), int(y), int(width), int(height)
        
        for i in range(x, x + width, step):
            obs.add((i, y))
        for i in range(y, y + height, step):
            obs.add((x + width, i))
        for i in range(x, x + width + step, step):
            obs.add((i, y + height))        
        for i in range(y, y + height + step, step):
            obs.add((x, i))
            
    def add_obstacles(self, obs, obstacles):
        for obstacle in obstacles:
            self.add_rectangular_obstacle(obs, **obstacle)

    def obs_map(self):
        """
        Initialize obstacles' positions
        :return: map of obstacles
        """

        x = self.x_range
        y = self.y_range
        obs = set()

        # Add borders as obstacles
        for i in range(x):
            obs.add((i, 0))
        for i in range(x):
            obs.add((i, y - 1))
        for i in range(y):
            obs.add((0, i))
        for i in range(y):
            obs.add((x - 1, i))

        # Border plot
        self.obs_bound = [
            [0, 0, 0.5, y],
            [0, y, x, 0.5],
            [0.5, 0, x, 0.5],
            [x, 0.5, 0.5, y]
        ] 
         
        if warehouse == 1: 
            # set common obstacle positions
            obstacles_to_add = [
            {'x': 10, 'y': 15, 'width': 80, 'height': 10},
            {'x': 10, 'y': 35, 'width': 80, 'height': 10},
            {'x': 10, 'y': 55, 'width': 80, 'height': 10},
            {'x': 10, 'y': 75, 'width': 80, 'height': 10},       
            ]     
        elif warehouse == 2:
            # set common obstacle positions
            obstacles_to_add = [
            {'x': 10, 'y': 15, 'width': 35, 'height': 10},
            {'x': 55, 'y': 15, 'width': 35, 'height': 10},
            {'x': 10, 'y': 35, 'width': 35, 'height': 10},
            {'x': 55, 'y': 35, 'width': 35, 'height': 10},
            {'x': 10, 'y': 55, 'width': 35, 'height': 10},
            {'x': 55, 'y': 55, 'width': 35, 'height': 10},
            {'x': 10, 'y': 75, 'width': 35, 'height': 10},
            {'x': 55, 'y': 75, 'width': 35, 'height': 10},       
            ] 
        elif warehouse == 3:
            # set common obstacle positions
            obstacles_to_add = [
            {'x': 15, 'y': 10, 'width': 10, 'height': 80},
            {'x': 35, 'y': 10, 'width': 10, 'height': 80},
            {'x': 55, 'y': 10, 'width': 10, 'height': 80},
            {'x': 75, 'y': 10, 'width': 10, 'height': 80},       
            ]
        elif warehouse == 4:
            # set common obstacle positions
            obstacles_to_add = [
            {'x': 15, 'y': 10, 'width': 10, 'height': 35},
            {'x': 15, 'y': 55, 'width': 10, 'height': 35},
            {'x': 35, 'y': 10, 'width': 10, 'height': 35},
            {'x': 35, 'y': 55, 'width': 10, 'height': 35},
            {'x': 55, 'y': 10, 'width': 10, 'height': 35},
            {'x': 55, 'y': 55, 'width': 10, 'height': 35},
            {'x': 75, 'y': 10, 'width': 10, 'height': 35},
            {'x': 75, 'y': 55, 'width': 10, 'height': 35},       
            ] 
        else:
            print("No warehouse environment obstacles added")    

        # Store obstacle values
        self.obs_rect = obstacles_to_add
        self.add_obstacles(obs,obstacles_to_add)
        # print(self.obs_rect)
        
        return obs

# def main():
#     env = Env()
#     print(env.obs)
        
    
# if __name__ == '__main__':
#     main()
    