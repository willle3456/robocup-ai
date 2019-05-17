#!/usr/bin/env python

import matplotlib.pyplot as plt
from matplotlib import collections as mc
import networkx as nx
from networkx import NetworkXNoPath
from path_planner import PathPlanner

class PlannerPlotter(object):
    def __init__(self, planner):
        self.planner = planner
        
        self.fig, self.ax = plt.subplots(1,1)
        self.graph_lines = []
        self.path_lines = []
        self.path_found = False
        self.setup_plotter()

    def setup_plotter(self):
        self.ax.set_xlim(self.planner.config_space.x_min, self.planner.config_space.x_max)
        self.ax.set_ylim(self.planner.config_space.y_min, self.planner.config_space.y_max)
        self.ax.hold(True)
        plt.show(False)
        plt.draw()

    def draw(self):
        if self.planner.found_goal:
            self.get_path_lines()
            if len(self.path_lines) < 1:
                self.path_found = False
                self.planner.graph.clear()
            else:
                path_lc = mc.LineCollection(self.path_lines, colors='m', linewidths=5)
                self.path_found = True
                self.ax.add_collection(path_lc)
        
        self.get_graph_lines()
        lc = mc.LineCollection(self.graph_lines, linewidths=0.5)
        self.ax.add_collection(lc)
        self.draw_obstacles()
        self.draw_start()
        self.draw_goal_area()
        self.fig.canvas.draw()

    def draw_obstacles(self):
        for o in self.planner.obstacles:
            tmp_circle = plt.Circle((o.x, o.y), self.planner.bot_size/2, color='r')
            self.ax.add_artist(tmp_circle)

    def draw_start(self):
        me_circle = plt.Circle((self.planner.start.x, self.planner.start.y), self.planner.bot_size/2, color='g')
        self.ax.add_artist(me_circle)

    def draw_goal_area(self):
        goal_circle = plt.Circle((self.planner.goal.x, self.planner.goal.y), (2 * self.planner.bot_size), color='#ffa500')
        self.ax.add_artist(goal_circle)
    
    def get_graph_lines(self):
        for u,v in self.planner.graph.edges:
            tmp_line = (u.x, u.y), (v.x, v.y)
            if tmp_line in self.graph_lines:
                print 'repeated line'
                continue
            self.graph_lines.append(tmp_line)

    def get_path(self):
        try:
            return nx.shortest_path(self.planner.graph, self.planner.start, self.planner.goal_node)
        except NetworkXNoPath:
            for node in list(self.planner.graph.nodes()):
                if node.dist(self.planner.start) < (1.25 * self.planner.bot_size):
                    try:
                        return nx.shortest_path(self.planner.graph, node, self.planner.goal_node)
                    except NetworkXNoPath:
                        pass

    def get_path_lines(self):
        path_nodes = self.get_path()
        
        if path_nodes is None:
            # exception, no path found...
            # might want to recalcuate from here
            self.path_lines = []
            return

        last_point = path_nodes[0]
        for pt in path_nodes:
            tmp_line = (last_point.x, last_point.y), (pt.x, pt.y)
            last_point = pt
            self.path_lines.append(tmp_line)

    def close(self):
        plt.close()
