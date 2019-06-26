#!/usr/bin/env  python
from basic_skills.player_data import Pose2D
import random

#TODO
# Uniform Sampler
def sample_uniform(config_space):
    x = random.uniform(config_space.x_min, config_space.x_max)
    y = random.uniform(config_space.y_min, config_space.y_max)
    q_rand = Pose2D(x, y, 0)
    return q_rand

#TODO
# Medial Sampler

#TODO
# Obstacle Sampler

#TODO
# Narrow Sampler

#TODO
# Bridge Sampler

#TODO
# Goal Biased Sampler

class ConfigSpace(object):
    def __init__(self, x_min, x_max, y_min, y_max):
        self.x_min = x_min
        self.x_max = x_max
        self.y_min = y_min
        self.y_max = y_max      
