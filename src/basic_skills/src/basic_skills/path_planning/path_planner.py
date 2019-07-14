import random
import numpy as np
import networkx as nx
from abc import ABCMeta, abstractmethod
import sampler
from robocup_msgs.msg import Graph

class PathPlanner(object):
    def __init__(self, start=None, goal=None, sampler_func=sampler.sample_uniform, apf=False):
        self.start = start
        self.goal = goal
        self.graph = nx.Graph()
        self.config_space = sampler.ConfigSpace(-4500.0, 4500.0, -3000.0, 3000.0)
        self.bot_size = 150.0
        self.obstacles = []
        self.found_goal = False
        self.goal_node = None
        self.path = []
        self.max_dist = self.get_max_dist()
        self.apf = apf
        self.sampler_func = sampler_func

        self.graph_data = Graph()

    def update_obstacles(self, obs):
        self.obstacles = obs

    def get_max_dist(self):
        top_left = np.array([self.config_space.x_min, self.config_space.y_max])
        bot_right = np.array([self.config_space.x_max, self.config_space.y_min])
        return np.linalg.norm(top_left - bot_right)

    def get_path(self):
        self.path = nx.shortest_path(self.graph, self.start, self.goal_node)
        return self.path

    def get_graph(self):
        return self.graph_data
    
    def is_graph_empty(self):
        a = len(self.graph.nodes()) == 0
        b = len(self.graph.edges()) == 0
        return a and b

    def sample(self):
        return self.sampler_func(self.config_space)

    @abstractmethod
    def plan(self):
        pass 

    @abstractmethod
    def rewire(self):
        pass

    @abstractmethod
    def avoid(self):
        pass

    def update(self):
        if self.apf:
            self.avoid()
        else:
            self.rewire()

    def run(self):
        if self.is_graph_empty():
            self.plan()
        else:
            self.update()
    
        path = self.get_path()
        self.graph_data.path = [pt.convert_to_pose() for pt in path]
        return path
