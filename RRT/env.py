"""
Environment for rrt_2D

Original Author: zhm-real
Source: https://github.com/zhm-real/PathPlanning

Modified by: Spyros Papasykiotis

Description:
Modified the environment to simulate a warehouse and added 4 warehouse presets.
"""
warehouse = 2

class Env:
    def __init__(self):
        self.x_range = (0, 100)
        self.y_range = (0, 100)
        self.obs_boundary = self.obs_boundary(self.x_range[1],self.y_range[1])
        self.obs_rectangle = self.obs_rectangle()

    @staticmethod
    def obs_boundary(x, y):
        obs_boundary = [
            [0, 0, 0.5, y],
            [0, y, x, 0.5],
            [0.5, 0, x, 0.5],
            [x, 0.5, 0.5, y]
        ]
        return obs_boundary
    @staticmethod
    def obs_rectangle():
        obs_rectangle = [       #x,y,width,height
            [10, 15, 80, 10],
            [10, 35, 80, 10],
            [10, 55, 80, 10],
            [10, 75, 80, 10],
        ]
        if warehouse == 1: 
            # set common obstacle positions
            obs_rectangle = [
                [10, 15, 80, 10],
                [10, 35, 80, 10],
                [10, 55, 80, 10],
                [10, 75, 80, 10],       
            ]     
        elif warehouse == 2:
            # set common obstacle positions
            obs_rectangle = [
                [10, 15, 35, 10],
                [55, 15, 35, 10],
                [10, 35, 35, 10],
                [55, 35, 35, 10],
                [10, 55, 35, 10],
                [55, 55, 35, 10],
                [10, 75, 35, 10],
                [55, 75, 35, 10],       
            ] 
        elif warehouse == 3:
            # set common obstacle positions
            obs_rectangle = [
                [15, 10, 10, 80],
                [35, 10, 10, 80],
                [55, 10, 10, 80],
                [75, 10, 10, 80],       
            ]
        elif warehouse == 4:
            # set common obstacle positions
            obs_rectangle = [
                [15, 10, 10, 35],
                [15, 55, 10, 35],
                [35, 10, 10, 35],
                [35, 55, 10, 35],
                [55, 10, 10, 35],
                [55, 55, 10, 35],
                [75, 10, 10, 35],
                [75, 55, 10, 35],       
            ] 
        else:
            print("No warehouse environment obstacles added")      
        return obs_rectangle

