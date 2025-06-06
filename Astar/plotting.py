"""
Plot tools 2D

Original Author: zhm-real
Source: https://github.com/zhm-real/PathPlanning

Modified by: Spyros Papasykiotis
"""

import matplotlib.pyplot as plt
import matplotlib.patches as patches
import env


class Plotting:
    def __init__(self, xI, xG):
        self.xI, self.xG = xI, xG
        self.env = env.Env()
        self.obs = self.env.obs_map()
        self.obs_rect = self.env.obs_rect
        self.obs_bound = self.env.obs_bound

    def update_obs(self, obs):
        self.obs = obs

    def animation(self, path, visited, name):
        self.plot_grid(name)
        self.plot_visited(visited)
        self.plot_path(path)
        plt.show()

    def animation_lrta(self, path, visited, name):
        self.plot_grid(name)
        cl = self.color_list_2()
        path_combine = []

        for k in range(len(path)):
            self.plot_visited(visited[k], cl[k])
            plt.pause(0.2)
            self.plot_path(path[k])
            path_combine += path[k]
            plt.pause(0.2)
        if self.xI in path_combine:
            path_combine.remove(self.xI)
        self.plot_path(path_combine)
        plt.show()

    def animation_ara_star(self, path, visited, name):
        self.plot_grid(name)
        cl_v, cl_p = self.color_list()

        for k in range(len(path)):
            self.plot_visited(visited[k], cl_v[k])
            self.plot_path(path[k], cl_p[k], True)
            plt.pause(0.5)

        plt.show()

    def animation_bi_astar(self, path, v_fore, v_back, name):
        self.plot_grid(name)
        self.plot_visited_bi(v_fore, v_back)
        self.plot_path(path)
        plt.show()

    def plot_grid(self, name, flag=False):
        obs_x = [x[0] for x in self.obs]
        obs_y = [x[1] for x in self.obs]

        plt.plot(obs_x, obs_y, "sk")
        plt.title(name)
        plt.axis("equal")
        if flag:
            plt.plot(self.xI[0], self.xI[1], "gs", markersize=5)
            plt.plot(self.xG[0], self.xG[1], "rs", markersize=5)

    def plot_grid_test(self, name, ax, flag=False):
        # fig, ax = plt.subplots()

        for (ox, oy, w, h) in self.obs_bound:
            ax.add_patch(
                patches.Rectangle(
                    (ox, oy), w, h,
                    edgecolor='black',
                    facecolor='black',
                    fill=True
                )
            )

        for obstacle in self.obs_rect:
            ox, oy, w, h = obstacle['x'], obstacle['y'], obstacle['width'], obstacle['height']
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
            plt.plot(self.xI[0], self.xI[1], "gs", markersize=5)
            plt.plot(self.xG[0], self.xG[1], "rs", markersize=5)
    
    def plot_path(self, path, line=None, cl=None, lb=None):
        path_x = [path[i][0] for i in range(len(path))]
        path_y = [path[i][1] for i in range(len(path))]
 
        plt.plot(path_x, path_y, linewidth='2',ls=line , color=cl, label=lb)
        
        plt.plot(self.xI[0], self.xI[1], "gs", markersize=5)
        plt.plot(self.xG[0], self.xG[1], "rs", markersize=5)

    def plot_visited(self, visited, cl='lightblue'):
        if self.xI in visited:
            visited.remove(self.xI)

        if self.xG in visited:
            visited.remove(self.xG)

        count = 0

        for x in visited:
            count += 1
            plt.plot(x[0], x[1], color=cl, marker='o')
            plt.gcf().canvas.mpl_connect('key_release_event', lambda event: [exit(0) if event.key == 'escape' else None])

            # if count < len(visited) / 3:
            #     length = 20
            # elif count < len(visited) * 2 / 3:
            #     length = 40
            # else:
            #     length = 60

            length = 120

            if count % length == 0:
                plt.pause(0.001)
        plt.pause(0.01)

    def plot_visited_bi(self, v_fore, v_back):
        if self.xI in v_fore:
            v_fore.remove(self.xI)

        if self.xG in v_back:
            v_back.remove(self.xG)

        len_fore, len_back = len(v_fore), len(v_back)

        for k in range(max(len_fore, len_back)):
            if k < len_fore:
                plt.plot(v_fore[k][0], v_fore[k][1], linewidth='3', color='gray', marker='o')
            if k < len_back:
                plt.plot(v_back[k][0], v_back[k][1], linewidth='3', color='cornflowerblue', marker='o')

            plt.gcf().canvas.mpl_connect('key_release_event',
                                         lambda event: [exit(0) if event.key == 'escape' else None])

            if k % 10 == 0:
                plt.pause(0.001)
        plt.pause(0.01)

    @staticmethod
    def color_list():
        cl_v = ['silver',
                'wheat',
                'lightskyblue',
                'royalblue',
                'slategray']
        cl_p = ['gray',
                'orange',
                'deepskyblue',
                'red',
                'm']
        return cl_v, cl_p

    @staticmethod
    def color_list_2():
        cl = ['silver',
              'steelblue',
              'dimgray',
              'cornflowerblue',
              'dodgerblue',
              'royalblue',
              'plum',
              'mediumslateblue',
              'mediumpurple',
              'blueviolet',
              ]
        return cl
