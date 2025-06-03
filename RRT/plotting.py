"""
Plotting tools for Sampling-based algorithms 2D

Original Author: zhm-real
Source: https://github.com/zhm-real/PathPlanning

Modified by: Spyros Papasykiotis

Description:
Modified to work with the grid world approach.
"""

import matplotlib.pyplot as plt
import matplotlib.patches as patches

import env


class Plotting:
    def __init__(self, x_start, x_goal):
        self.xI, self.xG = x_start, x_goal
        self.env = env.Env()
        self.obs_boundary = self.env.obs_boundary
        self.obs_rectangle = self.env.obs_rectangle
    
    def update_obs(self, obs_bound, obs_rec):
        
        self.obs_boundary = obs_bound
        self.obs_rectangle = obs_rec

    def animation(self, nodelist, path, name, animation=False):
        fig, ax = plt.subplots()
        self.plot_grid(name, ax)
        self.plot_visited(nodelist, animation)
        self.plot_path(path)
        plt.show()

    def animation_connect(self, V1, V2, path, name):
        self.plot_grid(name)
        self.plot_visited_connect(V1, V2)
        self.plot_path(path)

    def plot_grid(self, name, ax, flag=False):
        # fig, ax = plt.subplots()

        for (ox, oy, w, h) in self.obs_boundary:
            ax.add_patch(
                patches.Rectangle(
                    (ox, oy), w, h,
                    edgecolor='black',
                    facecolor='black',
                    fill=True
                )
            )

        for (ox, oy, w, h) in self.obs_rectangle:
            ax.add_patch(
                patches.Rectangle(
                    (ox, oy), w, h,
                    edgecolor='black',
                    facecolor='gray',
                    fill=True
                )
            )

        # Set the aspect ratio to be equal
        ax.set_aspect('equal')

        plt.title(name)
        plt.axis("off")
        if flag:
            plt.axis("equal") 

    @staticmethod
    def plot_visited(nodelist, animation):
        if animation:
            count = 0
            for node in nodelist:
                count += 1
                if node.parent:
                    plt.plot([node.parent.x, node.x], [node.parent.y, node.y], "-g")
                    plt.gcf().canvas.mpl_connect('key_release_event',
                                                 lambda event:
                                                 [exit(0) if event.key == 'escape' else None])
                    if count % 10 == 0:
                        plt.pause(0.001)
        else:
            for node in nodelist:
                if node.parent:
                    plt.plot([node.parent.x, node.x], [node.parent.y, node.y], "-g")

    @staticmethod
    def plot_visited_connect(V1, V2):
        len1, len2 = len(V1), len(V2)

        for k in range(max(len1, len2)):
            if k < len1:
                if V1[k].parent:
                    plt.plot([V1[k].x, V1[k].parent.x], [V1[k].y, V1[k].parent.y], "-g")
            if k < len2:
                if V2[k].parent:
                    plt.plot([V2[k].x, V2[k].parent.x], [V2[k].y, V2[k].parent.y], "-g")

            plt.gcf().canvas.mpl_connect('key_release_event',
                                         lambda event: [exit(0) if event.key == 'escape' else None])

            if k % 2 == 0:
                plt.pause(0.001)

        plt.pause(0.01)

    def plot_path(self, path,cl=None,line=None, lb=None):
        
        if len(path) != 0:
            plt.plot([x[0] for x in path], [x[1] for x in path], color=cl, ls=line, linewidth=2, label = lb)
            # plt.pause(0.01)
        
        plt.plot(self.xI[0], self.xI[1], "gs", markersize=5)
        plt.plot(self.xG[0], self.xG[1], "rs", markersize=5)
        

