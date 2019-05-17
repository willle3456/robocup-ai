import random
import numpy as np
import networkx as nx

class PathPlanner(object):
    def __init__(self):
        self.start = None
        self.goal = None
        self.graph = nx.Graph()
        self.config_space = ConfigSpace(-4500.0, 4500.0, -3000.0, 3000.0)
        self.bot_size = 15.0
        self.obstacles = []
        self.found_goal = False
        self.goal_node = None

    def update_obstacles(self, obs):
        self.obstacles = obs

    def point_collision(self, pt):
        for o in self.obstacles:
            if o.dist(pt) < (2 * self.bot_size):
               return True
        return False

    # not tested yet
    def intersect_lines(self, p1, p2, p3, p4):
        u_num = (((p1.x - p2.x) * (p1.y - p3.y)) - ((p1.y - p2.y)* (p1.x - p3.x)))
        t_num = (((p1.x - p3.x) * (p3.y - p4.y)) - ((p1.y - p3.y)* (p3.x - p4.x)))
        denom = (((p1.x - p2.x) * (p3.y - p4.y)) - ((p1.y - p2.y)*(p3.x - p4.x)))
        u = u_num / float(denom)
        t = t_num / float(denom)
        #print 'u: {0}\nt: {1}'.format(u, t)

        check_u = (u >= 0 and u <= 1)
        check_t = (t >= 0 and t <= 1)

        return check_u or check_t

    # not tested yet
    def line_collision(self, pt_1, pt_2):
        for o in self.obstacles:
            sides = self.get_sides(o)
            #print 'start check'
            for s in sides:
                if intersect_lines(pt_1, pt_2, s[0], s[1]):
                    #print 'end check: true'
                    return True
        #print 'end check: false'
        return False
               
    # not tested yet
    def get_sides(self, obs):
        offset = self.bot_size/2.0
        point_1 = Point(obs.x - offset, obs.y - offset)
        point_2 = Point(obs.x + offset, obs.y + offset)
        point_3 = Point(obs.x - offset, obs.y + offset)
        point_4 = Point(obs.x + offset, obs.y - offset)
        side_1 = [point_1, point_2]
        side_2 = [point_2, point_3]
        side_3 = [point_3, point_4]
        side_4 = [point_4, point_1]
        return side_1, side_2, side_3, side_4

    def get_random_pt(self):
        x = random.randint(self.config_space.x_min, self.config_space.x_max)
        y = random.randint(self.config_space.y_min, self.config_space.y_max)
        q_rand = Point(x, y)
        return q_rand

    # abstract function, must be implemented by path planners
    def run(self):
        pass 

class ConfigSpace(object):
    def __init__(self, x_min, x_max, y_min, y_max):
        self.x_min = x_min
        self.x_max = x_max
        self.y_min = y_min
        self.y_max = y_max      

class Point(object):
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def __str__(self):
        return 'Point:\nx: {0}, y:{1}\n'.format(self.x, self.y)
    
    def __repr__(self):
        return str(self)

    def convert_to_array(self):
        return np.array([self.x, self.y])

    def dist(self, other):
        pt_arr_1 = self.convert_to_array()
        pt_arr_2 = other.convert_to_array()
        return np.linalg.norm(pt_arr_1 - pt_arr_2)
