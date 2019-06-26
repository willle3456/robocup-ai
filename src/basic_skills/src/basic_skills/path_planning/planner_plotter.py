#!/usr/bin/env python

import matplotlib.pyplot as plt
from matplotlib import collections as mc
import networkx as nx
from networkx import NetworkXNoPath, NodeNotFound
from path_planner import PathPlanner
from basic_skills.player_data import Pose2D
import numpy as np

class PlannerPlotter(object):
    def __init__(self, planner):
        self.planner = planner
        self.fig, self.ax = plt.subplots(1,1)
        self.graph_lines = []
        self.path_lines = []
        self.traj_lines = []
        self.path_found = False
        self.robot = Pose2D(0,0,0)
        self.setup_plotter()

    def setup_plotter(self):
        self.ax.set_xlim(self.planner.config_space.x_min, self.planner.config_space.x_max)
        self.ax.set_ylim(self.planner.config_space.y_min, self.planner.config_space.y_max)
        self.ax.hold(True)
        #self.bkgnd = self.fig.canvas.copy_from_bbox(self.ax.bbox)
        plt.show(False)
        plt.draw()

    def draw(self):
        self.ax.clear()
        #self.fig.canvas.restore_region(self.bkgnd)
        if self.planner.found_goal:
            self.get_path_lines()
            #print len(list(self.planner.graph.nodes()))
            #print self.planner.graph.has_node(self.planner.goal_node)
            if len(self.path_lines) < 1:
                self.path_found = False
                self.planner.graph.clear()
            else:
                path_lc = mc.LineCollection(self.path_lines, colors='m', linewidths=5)
                traj_lc = mc.LineCollection(self.traj_lines, colors='k', linewidths=2)
                self.path_found = True
                self.ax.add_collection(path_lc)
                self.ax.add_collection(traj_lc)
        
        self.get_graph_lines()
        lc = mc.LineCollection(self.graph_lines, linewidths=0.5)
        self.ax.add_collection(lc)
        self.draw_nodes()
        self.draw_obstacles()
        self.draw_start()
        self.draw_current()
        self.draw_goal_area()
        #for a in self.ax.artists:
            #self.ax.draw_artist(a)
        #self.fig.canvas.blit(self.ax.bbox)
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
        self.graph_lines = []
        self.path_lines = []

    def draw_obstacles(self):
        count = 0
        for o in self.planner.obstacles:
            print count
            tmp_circle = plt.Circle((o.x, o.y), self.planner.bot_size/2, color='r')
            self.ax.add_artist(tmp_circle)
            count += 1

    def draw_nodes(self):
        for node in list(self.planner.graph.nodes()):
            dumb_circle = plt.Circle((node.x, node.y), 3, color='b')
            self.ax.add_artist(dumb_circle)

    def draw_start(self):
        me_circle = plt.Circle((self.planner.start.x, self.planner.start.y), self.planner.bot_size/2, color='g')
        self.ax.add_artist(me_circle)

    def draw_current(self):
        curr_circle = plt.Circle((self.robot.x, self.robot.y), self.planner.bot_size/2, color='g')
        self.ax.add_artist(curr_circle)

    def set_current(self, pt):
        self.robot = pt

    def draw_goal_area(self):
        goal_circle = plt.Circle((self.planner.goal.x, self.planner.goal.y), (3 * self.planner.bot_size), color='#ffa500')
        self.ax.add_artist(goal_circle)
    
    def get_graph_lines(self):
        #print len(self.planner.graph.edges())
        for u, v in self.planner.graph.edges():
            tmp_line = (u.x, u.y), (v.x, v.y)
            self.graph_lines.append(tmp_line)

    def get_path_lines(self):
        if self.planner.path is None:
            # exception, no path found...
            # might want to recalcuate from here
            self.path_lines = []
            return

        for pt_prev, pt_nxt in zip(self.planner.path, self.planner.path[1:]):
            tmp_line = (pt_prev.x, pt_prev.y), (pt_nxt.x, pt_nxt.y)
            self.path_lines.append(tmp_line)

    def get_traj(self, traj, delta_time):
        start = self.planner.start
        last_t = (start.x, start.y)
        #last_angle = 0.0
        for t in traj:
            #curr_angle = last_angle + t[1]
            #val_x = t[0] * np.cos(t[1])
            #val_y = t[0] * np.sin(t[1])
            #print val_x, val_y
            #tmp_x = last_t[0] + (val_x * delta_time * 7.5)
            #tmp_y = last_t[1] + (val_y * delta_time * 7.5)
            tmp_x = last_t[0] + (t[0] * 3.0)
            tmp_y = last_t[1] + (t[1] * 3.0)
            tmp_line = last_t, (tmp_x, tmp_y)
            last_t = (tmp_x, tmp_y)
            #last_angle = curr_angle
            self.traj_lines.append(tmp_line)
    
    def close(self):
        plt.close()
